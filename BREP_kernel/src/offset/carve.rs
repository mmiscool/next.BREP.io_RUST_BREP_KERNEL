//! CARVE the folded part out of a trim.
//!
//! `offset/regularity.rs` decides whether a trimmed region folds; `offset/
//! fold_locus.rs` traces the curve `1 − δ·κ = 0` that separates the part that
//! does from the part that does not.  This module is what the two are for: it
//! divides the trim along that curve, keeps every regular piece, and hands the
//! caller back the folded ones so they can be reported and the new boundaries
//! re-solved against their neighbours.
//!
//! # Where the crossings come from
//!
//! Not from intersecting two polylines.  The trim boundary and the locus are
//! both approximations in parameter space, and their polyline intersection
//! inherits both errors.  Instead the FIELD is evaluated along each trim
//! pcurve: a crossing is a zero of `1 − δ·κ(pcurve(t)) − level` in the
//! pcurve's own parameter, found by bisection and refined by Newton, so the
//! split point is exact in the one coordinate the split actually uses — the
//! parameter [`NurbsCurve::split`] is given.  The traced locus then only has to
//! supply the SHAPE of the bridge between two points it already passes through.
//!
//! # Any number of chords
//!
//! Each stretch of a branch that lies inside the trim, between two boundary
//! crossings, is a CHORD of the trim, and k non-crossing chords divide it into
//! k + 1 pieces.  One chord is the plain split: one regular piece, one folded.
//! TWO chords are a fold BAND across the trim — a torus offset past its inner
//! equator is the standard one — and they leave THREE pieces, the band in the
//! middle and a regular piece on either side of it.  A trim whose boundary
//! weaves across one fold parallel cuts as many chords as it has teeth, and
//! teeth across a band cut two each.  The division is built by recursion: take
//! one chord, it splits the cycle in two, every other chord lies wholly inside
//! one of them, and each side is the same problem with fewer chords.
//!
//! There is no cap on k.  There was one, two, until 2026-09-17, and its only
//! reason was that two was what the fixtures had measured; the recursion was
//! general, and so is the census below that verifies what it builds.  k is
//! bounded by the trim itself — every chord owns two crossings, and a crossing
//! is a sign change between two consecutive boundary samples, so k is at most
//! half the samples [`boundary_crossings`] takes — and each piece costs two
//! scans, so the work grows linearly in k.  Combs of three, four and eight
//! chords on a torus sheet read every kept piece on its Pappus volume and area
//! (`thicken.rs`, fixtures 17–19, and the 2026-09-17 carve record).
//!
//! # What it does not do
//!
//! A CLOSED branch — the fold cutting a HOLE out of the trim — is refused: the
//! kept region would carry a new inner loop whose offset image is a
//! self-touching rim, and nothing downstream re-solves that rim.  So is a
//! branch that crosses the boundary an odd number of times (a crossing the sign
//! change missed, or a tangency counted once), a crossing that bounds no stretch
//! of its branch inside the trim, and two branches that CROSS inside the trim.
//! Each is refused BY NAME with the counts it saw.  A carve that guesses at a
//! configuration it cannot resolve produces a trim that still validates and
//! bounds the wrong solid, which is the one outcome this family of code exists
//! to prevent.
//!
//! # The check that makes the result trustworthy
//!
//! The division is verified against the same scan that asked for it: EVERY
//! piece is censused, and each must come back either with no collapsed sample
//! or with nothing but collapsed samples, judged a band off the traced level so
//! the new boundaries themselves do not vote.  At least one piece of each kind
//! has to appear.  A division the scan disagrees with is refused, not shipped.

use crate::{KernelRefusal, KernelStage, OrRefuse};
use crate::offset_fold_locus::{fit_locus_pcurve, trace_fold_locus, FoldCurve};
use crate::offset_regularity::{
    fold_sample_at, region_grid, scan_offset_regularity, RegionGrid, ScanBudget, TrimRegion,
};
use crate::curve::KNOT_IDENTITY_TOL;
use crate::{NurbsCurve, NurbsSurface, Vec3};

/// Floor on how far off the traced level the verification scans judge, so that
/// samples sitting ON the new boundary belong to neither side's census. The band
/// actually used is the bridge's own measured residual where that is larger.
const VERIFICATION_BAND: f64 = 1e-9;
/// Newton's target for a crossing, in the fold factor.
const CROSSING_TOLERANCE: f64 = 1e-11;
/// What a refined crossing may still read before the bracket is called a JUMP
/// rather than a crossing. Nine decades below the O(1) residual a curvature
/// discontinuity leaves, and above the noise a fitted patch's second
/// derivatives carry near a knot line — see `fold_locus::CORRECTOR_STALL_CEILING`.
const CROSSING_CEILING: f64 = 1e-7;
/// How much nearer a crossing must be to its own branch than to any other
/// before the match is called unambiguous.
///
/// A crossing is a point its branch passes THROUGH, so the distance to it is a
/// fraction of the march's station spacing; the distance to a different branch
/// is the gap between two level curves of the same field, which is the width of
/// the fold band.  Two decades is far below that ratio on every fixture here
/// and far above the march's own placement error.
const ASSOCIATION_MARGIN: f64 = 1e-2;

/// One loop of a carved trim.
#[derive(Clone, Debug)]
pub(crate) struct CarvedLoop {
    pub(crate) pcurves: Vec<NurbsCurve>,
    /// Where the pcurve at the same index came from: `Some((loop, coedge))` for
    /// a surviving piece of the original trim, `None` for a piece of the FOLD
    /// LOCUS.
    ///
    /// The `None`s are the new boundary — the rim a caller has to re-solve
    /// against its neighbours. The `Some`s are what lets a caller keep the
    /// source edge's NAME on a piece that is still that edge: an offset
    /// carrier stamps `{name}_Offset` on every image edge it builds, and a
    /// carve that dropped the provenance would silently un-name half a face.
    pub(crate) origin: Vec<Option<(usize, usize)>>,
}

impl CarvedLoop {
    fn from_trim(loop_index: usize, pcurves: &[NurbsCurve]) -> Self {
        Self {
            pcurves: pcurves.to_vec(),
            origin: (0..pcurves.len())
                .map(|coedge| Some((loop_index, coedge)))
                .collect(),
        }
    }
}

/// A trim divided along the fold locus.
#[derive(Clone, Debug)]
pub(crate) struct CarvedTrim {
    /// The regular pieces, the ones to build on. ONE when a single locus branch
    /// splits the trim in two; TWO when a fold BAND crosses it, because the
    /// band's two boundary parallels leave regular skin on both sides of it; as
    /// many as the division leaves in general — three notches under one fold
    /// parallel are three.
    pub(crate) kept: Vec<Vec<CarvedLoop>>,
    /// The folded pieces, so a caller can report what it dropped and measure it.
    pub(crate) dropped: Vec<Vec<CarvedLoop>>,
    /// Parameter-space area of the kept pieces over that of the whole trim —
    /// a report quantity, never a gate.
    pub(crate) kept_fraction: f64,
}

/// The offset family's own trace switch, the same `BREP_OS_DEBUG` the shell and
/// the regularity scan read.
fn debug_enabled() -> bool {
    std::env::var("BREP_OS_DEBUG").is_ok_and(|value| !value.is_empty() && value != "0")
}

macro_rules! carve_debug {
    ($($arg:tt)*) => {
        if debug_enabled() {
            eprintln!($($arg)*);
        }
    };
}

/// Where a trim pcurve crosses the fold locus.
#[derive(Clone, Copy, Debug)]
struct Crossing {
    loop_index: usize,
    coedge_index: usize,
    parameter: f64,
    uv: [f64; 2],
}

/// How close the fold factor came to the level on the trim boundary WITHOUT
/// changing sign there — what a tangency looks like from the boundary's side.
#[derive(Clone, Copy, Debug)]
struct Approach {
    uv: [f64; 2],
    factor: f64,
}

/// One locus branch clipped to the trim: a CHORD of the trim's outer loop,
/// named by the two crossings it joins and carrying the bridge that realises it.
struct Chord {
    /// Indices into the outer loop's cyclically sorted crossing list, `from`
    /// before `to`.
    from: usize,
    to: usize,
    /// The bridge pcurve, running `from` → `to`.
    pcurve: NurbsCurve,
    /// How well that bridge names the locus — the verification band's source.
    residual: f64,
}

/// One stretch of a piece's boundary: the crossing it starts at, and the
/// pcurves running from there to the next stretch's crossing.
///
/// `None` in the second slot is a piece of the FOLD LOCUS; `Some(coedge)` is a
/// surviving piece of the original trim, which is what keeps the source edge's
/// name on it.
#[derive(Clone)]
struct Arc {
    start: usize,
    pieces: Vec<(NurbsCurve, Option<usize>)>,
}

/// Divide `loops` along the fold locus, keeping every regular piece.
///
/// `Ok(None)` when there is nothing to carve: no fold in the region at all, or
/// a region that folds at every sample — the all-or-nothing lane the callers
/// already have owns that one, and answering it here would be two mechanisms
/// for one question.
pub(crate) fn carve_folded_trim(
    surface: &NurbsSurface,
    loops: &[Vec<NurbsCurve>],
    displacements: &[f64],
    collapse_factor: f64,
    budget: ScanBudget,
) -> Result<Option<CarvedTrim>, KernelRefusal> {
    let region = TrimRegion::from_pcurve_loops(surface, loops)?;
    let scan = scan_offset_regularity(surface, &region, displacements, collapse_factor, budget)?;
    // Nothing to carve: no sample folds, not even between the nodes; or every
    // sample folds and no regular spot hides between them either.
    if scan.sampled == 0
        || (scan.collapsed == 0 && scan.between_collapsed == 0)
        || (scan.collapsed == scan.sampled && scan.between_regular == 0)
    {
        return Ok(None);
    }
    let locus = trace_fold_locus(surface, &region, displacements, collapse_factor, budget)?;
    // The boundary is read BEFORE the branches are judged, because a locus
    // TANGENT to the trim boundary leaves nothing for either to bracket — no
    // grid edge changes sign, so the trace finds no branch, and no pcurve
    // changes sign, so there is no crossing — and the only reading that
    // distinguishes it from "the fold is somewhere else entirely" is how close
    // the boundary came to the level.
    let (crossings, approach) =
        boundary_crossings(surface, loops, displacements, collapse_factor)?;
    let touching = |what: &str| {
        let where_it_touched = approach
            .map(|touch| {
                format!(
                    " Its closest approach to the level on the boundary is {:+.3e} at \
                     (u={:.6}, v={:.6})",
                    touch.factor, touch.uv[0], touch.uv[1]
                )
            })
            .unwrap_or_else(|| " No point of the boundary was readable".to_string());
        format!(
            "offset carve: the trim folds over {}/{} of its samples but {what} ({} branch(es) \
             traced over the parameter box, from {} bracket(s), {} crossing(s) on the \
             boundary).{where_it_touched} — a locus TANGENT to the trim boundary touches it and \
             turns back rather than passing through, so it cuts no chord and divides the trim \
             into nothing; a fold of zero width at one point of the rim is the gate's refusal \
             to make, and it names the curvature radius there",
            scan.collapsed,
            scan.sampled,
            locus.curves.len(),
            locus.brackets,
            crossings.len(),
        )
    };

    let branches = locus
        .curves
        .iter()
        .filter(|curve| curve.meets(&region))
        .collect::<Vec<_>>();
    if branches.is_empty() {
        return Err(KernelRefusal::unsupported(KernelStage::Classify, "carve_touch", touching("no fold locus branch lies inside it")));
    }
    if let Some(closed) = branches.iter().find(|branch| branch.closed) {
        let centre = closed.points.iter().fold([0.0f64; 2], |sum, point| {
            [
                sum[0] + point[0] / closed.points.len() as f64,
                sum[1] + point[1] / closed.points.len() as f64,
            ]
        });
        let extent = |axis: usize| {
            let (lo, hi) = closed.points.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), point| {
                (lo.min(point[axis]), hi.max(point[axis]))
            });
            hi - lo
        };
        return Err(KernelRefusal::unsupported(KernelStage::Classify, "carve_closed_branch", format!(
            "offset carve: a CLOSED fold locus branch lies inside the trim — {} stations around \
             (u={:.6}, v={:.6}), spanning {:.3e} × {:.3e} in (u, v) and not reaching the trim's \
             boundary — so the fold cuts a HOLE out of it rather than dividing it. The \
             kept region would be a trim with a new INNER loop whose image on the offset is a \
             self-touching rim, and nothing downstream re-solves that rim against its \
             neighbours; the carve refuses rather than ship a region that validates and bounds \
             the wrong solid",
            closed.points.len(),
            centre[0],
            centre[1],
            extent(0),
            extent(1)
        )));
    }

    let (target, ordered, chords) = chords_from_branches(
        surface,
        &region,
        &branches,
        &crossings,
        &touching,
        displacements,
        collapse_factor,
    )?;
    // The verification band is the BRIDGES' own measured residual, not a
    // constant. The new boundaries are interpolants, and the censuses below
    // re-read the field along them; judging those at 1e-9 while a bridge names
    // the locus to `residual` would count the boundary itself as folded and
    // refuse every carve whose locus is not an exact iso-line. Doubling is the
    // only slack, and the residual is reported so a caller can see it.
    let worst_residual = chords
        .iter()
        .map(|chord| chord.residual)
        .fold(0.0f64, f64::max);
    let band = (worst_residual * 2.0).max(VERIFICATION_BAND);
    carve_debug!(
        "carve: {} branch(es) bridge the locus to {worst_residual:.3e}; verification band \
         {band:.3e}",
        chords.len()
    );

    let chord_refs = chords.iter().collect::<Vec<_>>();
    let leaves = subdivide(initial_cycle(&loops[target], &ordered)?, &chord_refs)?;
    carve_debug!(
        "carve: {}/{} collapsed; {} chord(s) over {} crossing(s) on loop {target} divide the \
         trim into {} piece(s)",
        scan.collapsed,
        scan.sampled,
        chords.len(),
        ordered.len(),
        leaves.len()
    );

    let mut pieces = leaves
        .into_iter()
        .map(|arcs| {
            let mut carved = CarvedLoop {
                pcurves: Vec::with_capacity(arcs.len()),
                origin: Vec::with_capacity(arcs.len()),
            };
            for (pcurve, coedge) in arcs {
                carved.pcurves.push(pcurve);
                carved.origin.push(coedge.map(|coedge| (target, coedge)));
            }
            vec![carved]
        })
        .collect::<Vec<Vec<CarvedLoop>>>();

    // Any OTHER loop of the trim (a hole) belongs wholly to one piece.
    for (loop_index, loop_curves) in loops.iter().enumerate() {
        if loop_index == target {
            continue;
        }
        let sample = loop_sample(loop_curves)?;
        let mut hosts = Vec::new();
        for (index, piece) in pieces.iter().enumerate() {
            if region_of(surface, &piece[..1])?.contains(sample[0], sample[1]) {
                hosts.push(index);
            }
        }
        if hosts.len() != 1 {
            return Err(KernelRefusal::unsupported(KernelStage::Classify, "carve_hole_in_pieces", format!(
                "offset carve: trim loop {loop_index} lies inside {} of the {} pieces the fold \
                 locus divides the trim into — a hole cut by the fold is more than a division \
                 and the carve refuses",
                hosts.len(),
                pieces.len()
            )));
        }
        pieces[hosts[0]].push(CarvedLoop::from_trim(loop_index, loop_curves));
    }

    // Does the scan agree with the division? EVERY piece is censused and each
    // must come back WHOLLY regular or WHOLLY folded: naming a kept side and
    // testing only it - "the first piece if its census is clean, otherwise the
    // second" - would ship a division that separated nothing, or one whose
    // folded piece is too thin a sliver to catch a grid sample, on the strength
    // of a test that was never run. Each census is judged a band off the traced
    // level so the shared boundaries vote in neither.
    let mut kept = Vec::new();
    let mut dropped = Vec::new();
    for piece in pieces {
        let piece_region = region_of(surface, &piece)?;
        let regular_scan = scan_offset_regularity(
            surface,
            &piece_region,
            displacements,
            collapse_factor - band,
            budget,
        )?;
        let folded_scan = scan_offset_regularity(
            surface,
            &piece_region,
            displacements,
            collapse_factor + band,
            budget,
        )?;
        // Wholly, between the nodes as well: a kept piece holding a folded
        // island the census grid stepped over is not regular.
        let regular = regular_scan.sampled > 0
            && regular_scan.collapsed == 0
            && regular_scan.between_collapsed == 0;
        let folded = folded_scan.sampled > 0
            && folded_scan.collapsed == folded_scan.sampled
            && folded_scan.between_regular == 0;
        carve_debug!(
            "carve: piece over u[{:.6},{:.6}] v[{:.6},{:.6}] reads {}/{} collapsed above the \
             band and {}/{} below it — regular={regular} folded={folded}",
            piece_region.bounds()[0],
            piece_region.bounds()[1],
            piece_region.bounds()[2],
            piece_region.bounds()[3],
            regular_scan.collapsed,
            regular_scan.sampled,
            folded_scan.collapsed,
            folded_scan.sampled
        );
        match (regular, folded) {
            (true, false) => kept.push(piece),
            (false, true) => dropped.push(piece),
            _ => {
                return Err(KernelRefusal::ill_posed(KernelStage::Classify, "carve_mixed_piece", format!(
                    "offset carve: a piece of the division is neither wholly regular nor wholly \
                     folded — it reads {}/{} collapsed judged a band above the level (and {} more \
                     collapsed between the grid's nodes) and {}/{} judged a band below it (and {} \
                     regular between them), at the collapse factor {collapse_factor:.3e} ± \
                     {band:.3e}. The division is refused rather than shipped as a region that \
                     validates and bounds the wrong solid{}",
                    regular_scan.collapsed,
                    regular_scan.sampled,
                    regular_scan.between_collapsed,
                    folded_scan.collapsed,
                    folded_scan.sampled,
                    folded_scan.between_regular,
                    regular_scan
                        .worst
                        .map(|worst| format!(
                            ", worst 1 − δ·κ = {:.6e} at (u={:.6}, v={:.6})",
                            worst.factor, worst.u, worst.v
                        ))
                        .unwrap_or_default()
                )))
            }
        }
    }
    if kept.is_empty() || dropped.is_empty() {
        return Err(KernelRefusal::internal(KernelStage::Classify, "carve_empty_division", format!(
            "offset carve: the division left {} regular piece(s) and {} folded one(s) — the scan \
             says the trim folds over {}/{} of its samples, so both counts must be at least one, \
             and a division that drops everything or keeps everything is refused",
            kept.len(),
            dropped.len(),
            scan.collapsed,
            scan.sampled
        )));
    }

    let kept_area = kept.iter().map(|piece| parameter_area(piece)).sum::<f64>();
    let dropped_area = dropped
        .iter()
        .map(|piece| parameter_area(piece))
        .sum::<f64>();
    let total = kept_area + dropped_area;
    Ok(Some(CarvedTrim {
        kept,
        dropped,
        kept_fraction: if total > 0.0 { kept_area / total } else { 0.0 },
    }))
}

/// Every zero of the fold factor along the trim's own pcurves.
fn boundary_crossings(
    surface: &NurbsSurface,
    loops: &[Vec<NurbsCurve>],
    displacements: &[f64],
    level: f64,
) -> Result<(Vec<Crossing>, Option<Approach>), KernelRefusal> {
    let mut crossings = Vec::new();
    // The closest the boundary came to the level without crossing it. A locus
    // TANGENT to the trim boundary leaves no sign change to bracket, so the
    // crossing list alone cannot tell "the locus never reaches the boundary"
    // from "it touches and turns back"; this is the reading that can.
    let mut approach: Option<Approach> = None;
    for (loop_index, loop_curves) in loops.iter().enumerate() {
        for (coedge_index, pcurve) in loop_curves.iter().enumerate() {
            let [t0, t1] = pcurve.domain().or_refuse(KernelStage::Classify, "domain")?;
            if !(t1 > t0) {
                continue;
            }
            let spans = distinct_knots(&pcurve.knots, t0, t1).max(1);
            let steps = (32 * spans).clamp(64, 1024);
            let factor_at = |t: f64| -> Result<Option<f64>, KernelRefusal> {
                let uv = pcurve.evaluate(t).or_refuse(KernelStage::Classify, "evaluate")?;
                Ok(fold_sample_at(surface, uv.x, uv.y, displacements)
                    .map(|sample| sample.factor - level))
            };
            let mut previous: Option<(f64, f64)> = None;
            for step in 0..=steps {
                let t = t0 + (t1 - t0) * step as f64 / steps as f64;
                let Some(value) = factor_at(t)? else {
                    previous = None;
                    continue;
                };
                if approach.is_none_or(|best| value.abs() < best.factor.abs()) {
                    let uv = pcurve.evaluate(t).or_refuse(KernelStage::Classify, "evaluate")?;
                    approach = Some(Approach {
                        uv: [uv.x, uv.y],
                        factor: value,
                    });
                }
                if let Some((previous_t, previous_value)) = previous {
                    if (value < 0.0) != (previous_value < 0.0) {
                        let (parameter, residual) =
                            refine_crossing(&factor_at, previous_t, previous_value, t, value)?;
                        if residual.abs() > CROSSING_CEILING {
                            return Err(KernelRefusal::unsupported(KernelStage::Classify, "carve_factor_jump", format!(
                                "offset carve: the fold factor JUMPS across t={parameter:.9} on \
                                 the trim boundary (residual {residual:+.3e}) — the trim crosses \
                                 a curvature discontinuity, not the fold locus, and there is no \
                                 point on 1 − δ·κ = {level:.3e} to split it at"
                            )));
                        }
                        let uv = pcurve.evaluate(parameter).or_refuse(KernelStage::Classify, "evaluate")?;
                        // A crossing AT a trim vertex cannot be split at: the
                        // piece on one side would have no length, and
                        // `NurbsCurve::split` refuses it with a message that
                        // names neither the fold nor the vertex. Measured on
                        // the torus sheet with a trim corner placed on the
                        // carved parallel itself (`thicken.rs`, fixture 20).
                        if parameter <= t0 + KNOT_IDENTITY_TOL || parameter >= t1 - KNOT_IDENTITY_TOL
                        {
                            return Err(KernelRefusal::unsupported(KernelStage::Classify, "carve_crossing_at_vertex", format!(
                                "offset carve: the fold locus crosses the trim boundary AT a \
                                 vertex — (u={:.6}, v={:.6}), t={parameter:.12} on a pcurve over \
                                 [{t0}, {t1}] — so there is no boundary on either side of the \
                                 crossing to split; a trim corner sitting on 1 − δ·κ = \
                                 {level:.3e} is refused rather than divided with a zero-length \
                                 piece",
                                uv.x, uv.y
                            )));
                        }
                        crossings.push(Crossing {
                            loop_index,
                            coedge_index,
                            parameter,
                            uv: [uv.x, uv.y],
                        });
                    }
                }
                previous = Some((t, value));
            }
        }
    }
    Ok((crossings, approach))
}

fn refine_crossing(
    factor_at: &dyn Fn(f64) -> Result<Option<f64>, KernelRefusal>,
    mut lo: f64,
    mut value_lo: f64,
    mut hi: f64,
    mut value_hi: f64,
) -> Result<(f64, f64), KernelRefusal> {
    for _ in 0..100 {
        let mid = 0.5 * (lo + hi);
        let Some(value) = factor_at(mid)? else {
            return Ok((mid, f64::INFINITY));
        };
        if value.abs() <= CROSSING_TOLERANCE {
            return Ok((mid, value));
        }
        if (value < 0.0) == (value_lo < 0.0) {
            lo = mid;
            value_lo = value;
        } else {
            hi = mid;
            value_hi = value;
        }
        if (hi - lo).abs() <= 1e-15 * (1.0 + lo.abs()) {
            break;
        }
    }
    Ok(if value_lo.abs() <= value_hi.abs() {
        (lo, value_lo)
    } else {
        (hi, value_hi)
    })
}

fn distinct_knots(knots: &[f64], lo: f64, hi: f64) -> usize {
    let epsilon = 1e-12 * (hi - lo).abs().max(1.0);
    let mut count = 1usize;
    let mut last = lo;
    for &knot in knots {
        if knot > lo + epsilon && knot < hi - epsilon && knot > last + epsilon {
            count += 1;
            last = knot;
        }
    }
    count
}

/// Every locus branch as a CHORD of one trim loop, with the bridge that
/// realises it.
///
/// Returns the target loop, its crossings in cyclic order, and one chord per
/// branch indexing into that order.
///
/// A crossing is matched to a branch by NEAREST STATION, and the match has to
/// be unambiguous: a crossing is a point the branch passes through, so its
/// distance to the branch it belongs to is a fraction of the march's own
/// station spacing, while its distance to any OTHER branch is the gap between
/// two level curves. A match that is not clear by [`ASSOCIATION_MARGIN`] is a
/// refusal, not a guess — picking the nearer of two comparable distances would
/// bridge the wrong pair and divide the trim along a curve the fold is not on.
fn chords_from_branches(
    surface: &NurbsSurface,
    region: &TrimRegion,
    branches: &[&FoldCurve],
    crossings: &[Crossing],
    touching: &dyn Fn(&str) -> String,
    displacements: &[f64],
    level: f64,
) -> Result<(usize, Vec<Crossing>, Vec<Chord>), KernelRefusal> {
    if crossings.is_empty() {
        return Err(KernelRefusal::unsupported(KernelStage::Classify, "carve_touch", touching(
            "the fold factor never changes sign on the trim's own boundary",
        )));
    }
    // Which branch does each crossing belong to, and is the answer clear?
    let mut owner = Vec::with_capacity(crossings.len());
    for crossing in crossings {
        let mut distances = branches
            .iter()
            .enumerate()
            .map(|(index, branch)| (index, station_distance(branch, crossing.uv)))
            .collect::<Vec<_>>();
        distances.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));
        let (best, best_distance) = distances[0];
        if let Some(&(_, runner_up)) = distances.get(1) {
            if best_distance > runner_up * ASSOCIATION_MARGIN {
                return Err(KernelRefusal::ill_posed(KernelStage::Classify, "carve_ambiguous_branch", format!(
                    "offset carve: the crossing at (u={:.6}, v={:.6}) is {best_distance:.3e} from \
                     one fold locus branch and {runner_up:.3e} from another — the two are not \
                     separated by the factor {ASSOCIATION_MARGIN} this needs to say which branch \
                     the trim boundary crosses there, and the carve refuses rather than bridge \
                     the wrong pair",
                    crossing.uv[0], crossing.uv[1]
                )));
            }
        }
        carve_debug!(
            "carve: crossing loop {} coedge {} t={:.9} at (u={:.9}, v={:.9}) belongs to branch \
             {best} at {best_distance:.3e}",
            crossing.loop_index,
            crossing.coedge_index,
            crossing.parameter,
            crossing.uv[0],
            crossing.uv[1]
        );
        owner.push(best);
    }

    // Every branch has to cross the boundary an EVEN number of times: it comes
    // in and it goes out. An odd count is a crossing the field's sign change
    // missed or a tangency counted once, and either way the pairing below would
    // be guessing.
    let target = crossings[0].loop_index;
    for (index, branch) in branches.iter().enumerate() {
        let count = owner.iter().filter(|&&which| which == index).count();
        if count < 2 || count % 2 != 0 {
            return Err(KernelRefusal::ill_posed(KernelStage::Classify, "carve_odd_crossings", format!(
                "offset carve: fold locus branch {index} ({} stations, {}) crosses the trim \
                 boundary {count} time(s). A branch enters and leaves, so the count has to be \
                 EVEN and at least two; an odd one is a crossing the sign change missed or a \
                 tangency counted once, and pairing them up would be a guess",
                branch.points.len(),
                if branch.closed { "closed" } else { "open" },
            )));
        }
    }
    if let Some(stray) = crossings.iter().find(|crossing| crossing.loop_index != target) {
        return Err(KernelRefusal::unsupported(KernelStage::Classify, "carve_changes_topology", format!(
            "offset carve: the fold locus crosses trim loop {target} and trim loop {} — it \
             separates a hole from its outer boundary, which changes the region's topology \
             rather than dividing it, and the carve refuses",
            stray.loop_index
        )));
    }

    // Cyclic order along the target loop is what the division walks.
    let mut order = (0..crossings.len()).collect::<Vec<_>>();
    order.sort_by(|&a, &b| {
        (crossings[a].coedge_index, crossings[a].parameter)
            .partial_cmp(&(crossings[b].coedge_index, crossings[b].parameter))
            .unwrap_or(std::cmp::Ordering::Equal)
    });
    let ordered = order
        .iter()
        .map(|&index| crossings[index])
        .collect::<Vec<_>>();
    let rank = {
        let mut rank = vec![0usize; crossings.len()];
        for (position, &index) in order.iter().enumerate() {
            rank[index] = position;
        }
        rank
    };

    // One chord per stretch of a branch that lies INSIDE the trim.
    //
    // A branch crossing the boundary twice contributes one. A branch crossing it
    // FOUR times enters and leaves twice, and the two stretches inside it are
    // two chords of ONE branch — a wavy trim over a single fold parallel is that
    // shape, and it divides the trim exactly the way two separate branches
    // would. Which stretches are inside is ASKED of the region rather than
    // assumed to alternate from the first: the branch is marched over the trim's
    // bounding BOX, so on a trim that is not a rectangle its first stretch can
    // lie outside.
    let mut chords = Vec::new();
    for (index, branch) in branches.iter().enumerate() {
        let mut along = (0..crossings.len())
            .filter(|&crossing| owner[crossing] == index)
            .map(|crossing| (station_foot(branch, crossings[crossing].uv).1, rank[crossing]))
            .collect::<Vec<_>>();
        along.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
        // How many stretches inside the trim each crossing ends. A crossing the
        // branch passes THROUGH ends exactly one: the stretch on one side of it
        // is inside and the stretch on the other is not.
        let mut bounded = vec![0usize; along.len()];
        for (step, pair) in along.windows(2).enumerate() {
            let middle = branch_point_at(branch, 0.5 * (pair[0].0 + pair[1].0));
            if !region.contains(middle[0], middle[1]) {
                continue;
            }
            bounded[step] += 1;
            bounded[step + 1] += 1;
            let (from, to) = (pair[0].1.min(pair[1].1), pair[0].1.max(pair[1].1));
            let (pcurve, residual) = locus_bridge(
                surface,
                branch,
                displacements,
                level,
                ordered[from].uv,
                ordered[to].uv,
            )?;
            carve_debug!(
                "carve: branch {index} chord {from}..{to} over stations {:.3}..{:.3}, bridge \
                 residual {residual:.3e}",
                pair[0].0,
                pair[1].0
            );
            chords.push(Chord {
                from,
                to,
                pcurve,
                residual,
            });
        }
        if let Some(loose) = bounded.iter().position(|&count| count != 1) {
            let crossing = ordered[along[loose].1];
            return Err(KernelRefusal::ill_posed(KernelStage::Classify, "carve_crossing_pairing", if bounded[loose] == 0 {
                format!(
                    "offset carve: crossing {} of fold locus branch {index}, at (u={:.6}, \
                     v={:.6}), bounds no stretch of it that lies inside the trim — the boundary \
                     meets the level there and the locus leaves again without entering, which is \
                     a touch rather than a crossing, and the carve refuses rather than leave a \
                     chord dangling",
                    along[loose].1,
                    crossing.uv[0],
                    crossing.uv[1]
                )
            } else {
                format!(
                    "offset carve: crossing {} of fold locus branch {index}, at (u={:.6}, \
                     v={:.6}), ends a stretch of it inside the trim on BOTH sides — the branch \
                     does not leave the trim there, so either the boundary touches it from inside \
                     or a crossing beside this one was missed; two chords would share one \
                     endpoint, and the carve refuses rather than divide on crossings that do not \
                     alternate",
                    along[loose].1,
                    crossing.uv[0],
                    crossing.uv[1]
                )
            }));
        }
    }
    carve_debug!(
        "carve: {} chord(s) over {} branch(es) from {} crossing(s)",
        chords.len(),
        branches.len(),
        crossings.len()
    );
    // Non-crossing is what makes the recursion below a division: two chords of
    // the same level set can only meet at a saddle, and a saddle inside the
    // trim is a configuration with no unambiguous division.
    for (a, first) in chords.iter().enumerate() {
        for second in &chords[a + 1..] {
            // Strictly inside the arc `first.from` → `first.to`, walking the
            // cycle forward. Two chords are non-crossing when the second lies
            // WHOLLY inside that arc or wholly outside it; exactly one endpoint
            // inside is the interleaving that makes them cross.
            let inside = |crossing: usize| {
                (first.from < crossing) != (first.to < crossing)
            };
            if inside(second.from) != inside(second.to) {
                return Err(KernelRefusal::ill_posed(KernelStage::Classify, "carve_branches_cross", format!(
                    "offset carve: two fold locus branches CROSS inside the trim — their \
                     crossings interleave along the boundary as {}..{} and {}..{} — so the level \
                     set has a saddle there and the division is ambiguous; the carve refuses",
                    first.from, first.to, second.from, second.to
                )));
            }
        }
    }
    Ok((target, ordered, chords))
}

/// Distance from a point to a branch's marched polyline.
fn station_distance(branch: &FoldCurve, target: [f64; 2]) -> f64 {
    station_foot(branch, target).0
}

/// The same, plus WHERE along the polyline the foot of that perpendicular lies,
/// in station units (`3.5` is halfway between stations 3 and 4).
///
/// That position is what orders a branch's crossings ALONG the branch, which is
/// a different order from their order around the trim boundary and the only one
/// that says which stretches of the branch lie inside the trim.
fn station_foot(branch: &FoldCurve, target: [f64; 2]) -> (f64, f64) {
    let mut best = (f64::INFINITY, 0.0);
    for (index, window) in branch.points.windows(2).enumerate() {
        let (a, b) = (window[0], window[1]);
        let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
        let length = dx * dx + dy * dy;
        let t = if length > 0.0 {
            (((target[0] - a[0]) * dx + (target[1] - a[1]) * dy) / length).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let distance = (a[0] + t * dx - target[0]).hypot(a[1] + t * dy - target[1]);
        if distance < best.0 {
            best = (distance, index as f64 + t);
        }
    }
    if branch.points.len() == 1 {
        best = (
            (branch.points[0][0] - target[0]).hypot(branch.points[0][1] - target[1]),
            0.0,
        );
    }
    best
}

/// The branch's point at a fractional station position.
fn branch_point_at(branch: &FoldCurve, position: f64) -> [f64; 2] {
    let last = branch.points.len().saturating_sub(1);
    let clamped = position.clamp(0.0, last as f64);
    let index = (clamped.floor() as usize).min(last.saturating_sub(1));
    let t = clamped - index as f64;
    let (a, b) = (branch.points[index], branch.points[(index + 1).min(last)]);
    [a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])]
}

/// The target loop as a cyclic sequence of boundary stretches, one between each
/// pair of consecutive crossings.
fn initial_cycle(loop_curves: &[NurbsCurve], ordered: &[Crossing]) -> Result<Vec<Arc>, KernelRefusal> {
    let count = ordered.len();
    let mut cycle = Vec::with_capacity(count);
    for index in 0..count {
        let chain = boundary_chain(loop_curves, ordered[index], ordered[(index + 1) % count])?;
        cycle.push(Arc {
            start: index,
            pieces: chain
                .into_iter()
                .map(|(pcurve, coedge)| (pcurve, Some(coedge)))
                .collect(),
        });
    }
    Ok(cycle)
}

/// Divide a cyclic boundary along its chords, one chord at a time.
///
/// k non-crossing chords divide a disc into k + 1 pieces, and the recursion is
/// what makes that constructive rather than asserted: taking one chord splits
/// the cycle into two cycles, every remaining chord has BOTH endpoints on one
/// of them (that is what non-crossing means, and it is checked), and each side
/// is the same problem with fewer chords. A chord whose endpoints land on
/// different sides is the refusal in [`chords_from_branches`] arriving late,
/// and it is refused here too rather than silently dropped.
fn subdivide(
    cycle: Vec<Arc>,
    chords: &[&Chord],
) -> Result<Vec<Vec<(NurbsCurve, Option<usize>)>>, KernelRefusal> {
    let Some((chord, rest)) = chords.split_first() else {
        return Ok(vec![cycle
            .into_iter()
            .flat_map(|arc| arc.pieces)
            .collect::<Vec<_>>()]);
    };
    let count = cycle.len();
    let position = |crossing: usize| cycle.iter().position(|arc| arc.start == crossing);
    let (Some(at_from), Some(at_to)) = (position(chord.from), position(chord.to)) else {
        return Err(KernelRefusal::internal(KernelStage::Classify, "carve_chord_junctions", format!(
            "offset carve: a fold locus chord joins crossings {} and {}, which are not both \
             junctions of the piece being divided — the branches do not divide the trim into \
             nested pieces and the carve refuses",
            chord.from, chord.to
        )));
    };
    let span = |start: usize, end: usize| {
        let mut arcs = Vec::new();
        let mut index = start;
        while index != end {
            arcs.push(cycle[index].clone());
            index = (index + 1) % count;
        }
        arcs
    };
    // The piece running `from` → `to` along the boundary closes with the chord
    // taken `to` → `from`, and the other way round.
    let mut near = span(at_from, at_to);
    near.push(Arc {
        start: chord.to,
        pieces: vec![(chord.pcurve.reversed().or_refuse(KernelStage::Classify, "reversed")?, None)],
    });
    let mut far = span(at_to, at_from);
    far.push(Arc {
        start: chord.from,
        pieces: vec![(chord.pcurve.clone(), None)],
    });

    let (mut near_chords, mut far_chords) = (Vec::new(), Vec::new());
    for chord in rest {
        let holds = |side: &[Arc], crossing: usize| side.iter().any(|arc| arc.start == crossing);
        match (
            holds(&near, chord.from) && holds(&near, chord.to),
            holds(&far, chord.from) && holds(&far, chord.to),
        ) {
            (true, false) => near_chords.push(*chord),
            (false, true) => far_chords.push(*chord),
            _ => {
                return Err(KernelRefusal::ill_posed(KernelStage::Classify, "carve_branches_cross", format!(
                    "offset carve: the fold locus chord joining crossings {} and {} straddles \
                     the division made by another branch — the branches CROSS inside the trim \
                     and the carve refuses rather than pick a division",
                    chord.from, chord.to
                )))
            }
        }
    }
    let mut pieces = subdivide(near, &near_chords)?;
    pieces.extend(subdivide(far, &far_chords)?);
    Ok(pieces)
}

/// The stretch of the trim boundary from one crossing to the next, walking the
/// loop in its own order.
fn boundary_chain(
    loop_curves: &[NurbsCurve],
    from: Crossing,
    to: Crossing,
) -> Result<Vec<(NurbsCurve, usize)>, KernelRefusal> {
    let count = loop_curves.len();
    let mut chain = Vec::new();
    if from.coedge_index == to.coedge_index {
        let pcurve = &loop_curves[from.coedge_index];
        if to.parameter > from.parameter {
            // Both crossings on one pcurve, the chain is the piece between.
            let (_, after) = pcurve.split(from.parameter).or_refuse(KernelStage::Classify, "split")?;
            let (middle, _) = after.split(to.parameter).or_refuse(KernelStage::Classify, "split")?;
            chain.push((middle, from.coedge_index));
            return Ok(chain);
        }
        // The chain wraps: the tail of this pcurve, every other pcurve, then
        // its head.
        let (_, tail) = pcurve.split(from.parameter).or_refuse(KernelStage::Classify, "split")?;
        chain.push((tail, from.coedge_index));
        for offset in 1..count {
            let index = (from.coedge_index + offset) % count;
            chain.push((loop_curves[index].clone(), index));
        }
        let (head, _) = pcurve.split(to.parameter).or_refuse(KernelStage::Classify, "split")?;
        chain.push((head, to.coedge_index));
        return Ok(chain);
    }
    let (_, tail) = loop_curves[from.coedge_index].split(from.parameter).or_refuse(KernelStage::Classify, "split")?;
    chain.push((tail, from.coedge_index));
    let mut index = (from.coedge_index + 1) % count;
    while index != to.coedge_index {
        chain.push((loop_curves[index].clone(), index));
        index = (index + 1) % count;
    }
    let (head, _) = loop_curves[to.coedge_index].split(to.parameter).or_refuse(KernelStage::Classify, "split")?;
    chain.push((head, to.coedge_index));
    Ok(chain)
}

/// The piece of the traced locus between two points it passes through, as a
/// pcurve running `from` → `to`.
///
/// The endpoints are the CROSSINGS, not the nearest marched station: the
/// crossing is where the split actually happens and the bridge has to start and
/// end exactly there or the carved loop does not close.
fn locus_bridge(
    surface: &NurbsSurface,
    branch: &FoldCurve,
    displacements: &[f64],
    level: f64,
    from: [f64; 2],
    to: [f64; 2],
) -> Result<(NurbsCurve, f64), KernelRefusal> {
    let nearest = |target: [f64; 2]| -> usize {
        let mut best = 0usize;
        let mut best_distance = f64::INFINITY;
        for (index, point) in branch.points.iter().enumerate() {
            let distance = (point[0] - target[0]).hypot(point[1] - target[1]);
            if distance < best_distance {
                best_distance = distance;
                best = index;
            }
        }
        best
    };
    let (start, end) = (nearest(from), nearest(to));
    let mut stations: Vec<[f64; 2]> = if start <= end {
        branch.points[start..=end].to_vec()
    } else {
        branch.points[end..=start].iter().rev().copied().collect()
    };
    if stations.len() < 2 {
        return Err(KernelRefusal::unsupported(KernelStage::Classify, "carve_single_point_locus", "offset carve: the fold locus between the two crossings is a single point"));
    }
    // REPLACE the end stations with the exact crossings; do not prepend them.
    //
    // The crossing and the marched station nearest it are the same point of the
    // locus to ~1e-9, and keeping both leaves a chord-length span of 1e-9 next
    // to spans of 5e-2.  A cubic interpolant through that ratio does not sag a
    // little, it RINGS: measured on a torus whose locus is an exact iso-v line,
    // the bridge wandered v by 1.2e-3 either side of the crossing — enough to
    // leave a strip of folded parameter inside the kept region and to make the
    // carve refuse its own split.
    let last = stations.len() - 1;
    stations[0] = from;
    stations[last] = to;
    // The same reasoning one station in: a marched station that the exact
    // endpoint has overtaken, or nearly, is the same ringing.
    let spacing = {
        let mut lengths = stations
            .windows(2)
            .map(|pair| (pair[1][0] - pair[0][0]).hypot(pair[1][1] - pair[0][1]))
            .collect::<Vec<_>>();
        lengths.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        lengths[lengths.len() / 2]
    };
    let floor = spacing * 0.25;
    let mut cleaned: Vec<[f64; 2]> = Vec::with_capacity(stations.len());
    for (index, station) in stations.iter().enumerate() {
        let is_end = index == 0 || index == last;
        if !is_end
            && cleaned.last().is_some_and(|previous| {
                (previous[0] - station[0]).hypot(previous[1] - station[1]) < floor
            })
        {
            continue;
        }
        cleaned.push(*station);
    }
    // The last station is never dropped above, so a short FINAL span is trimmed
    // by removing the one before it instead.
    if cleaned.len() > 2 {
        let tail = cleaned.len() - 1;
        let gap = (cleaned[tail][0] - cleaned[tail - 1][0])
            .hypot(cleaned[tail][1] - cleaned[tail - 1][1]);
        if gap < floor {
            cleaned.remove(tail - 1);
        }
    }
    if cleaned.len() < 2 {
        return Err(KernelRefusal::unsupported(KernelStage::Classify, "carve_single_point_locus", "offset carve: the fold locus between the two crossings is a single point"));
    }
    fit_locus_pcurve(surface, displacements, level, &cleaned, false)
}


/// One point the DROPPED part of a trim sweeps along its own normal.
#[derive(Clone, Copy, Debug)]
pub(crate) struct SweptPoint {
    pub(crate) uv: [f64; 2],
    /// How far along the displacement, as a fraction in (0, 1].
    pub(crate) fraction: f64,
    pub(crate) displacement: f64,
    pub(crate) point: Vec3,
}

/// Every point the dropped pieces sweep, for a caller to classify against what
/// it actually built.
///
/// **A carve may drop a part of the trim only when that part's own normals
/// sweep material the kept construction contains.** That is the whole rule, and
/// it is a property of the geometry rather than of the piece count: dropping
/// the folded part is sound when the material it would have contributed is
/// already inside the kept result, and unsound when it is not.
///
/// * An apex cone shelled through its base passes. The dropped sliver near the
///   apex sweeps INWARD, into source material above the cavity's own apex,
///   which the shell keeps.
/// * A torus sheet thickened OUTWARD past its tube radius fails, whether the
///   fold is a band or a single parallel. At the inner equator the outward
///   normal points at the axis: the segment reaches the axis part way along and
///   comes out the far side, sweeping material at the opposite azimuth that no
///   kept piece contains.
///
/// The sampling is the dropped region's own scan grid — the same population the
/// census read — times five fractions of each displacement, the far end
/// included. It is a bound, not a proof, in exactly the way the regularity scan
/// is; what it catches is a dropped part whose sweep LEAVES, which is a
/// wholesale departure rather than a sliver.
pub(crate) fn dropped_sweep(
    surface: &NurbsSurface,
    dropped: &[Vec<CarvedLoop>],
    displacements: &[f64],
    budget: ScanBudget,
) -> Result<Vec<SweptPoint>, KernelRefusal> {
    const FRACTIONS: [f64; 5] = [0.2, 0.4, 0.6, 0.8, 1.0];
    let reach = displacements
        .iter()
        .fold(0.0f64, |worst, distance| worst.max(distance.abs()));
    let mut swept = Vec::new();
    for piece in dropped {
        let region = region_of(surface, piece)?;
        let RegionGrid {
            samples_u,
            samples_v,
        } = region_grid(surface, region.bounds(), reach, budget);
        for &u in &samples_u {
            for &v in &samples_v {
                if !region.contains(u, v) {
                    continue;
                }
                let Ok(base) = surface.evaluate(u, v) else {
                    continue;
                };
                let Ok(normal) = surface.normal(u, v) else {
                    continue;
                };
                for &displacement in displacements {
                    if displacement == 0.0 {
                        continue;
                    }
                    for fraction in FRACTIONS {
                        swept.push(SweptPoint {
                            uv: [u, v],
                            fraction,
                            displacement,
                            point: base.add(normal.scale(displacement * fraction)),
                        });
                    }
                }
            }
        }
    }
    Ok(swept)
}

fn region_of(surface: &NurbsSurface, side: &[CarvedLoop]) -> Result<TrimRegion, KernelRefusal> {
    let loops = side
        .iter()
        .map(|carved| carved.pcurves.clone())
        .collect::<Vec<_>>();
    TrimRegion::from_pcurve_loops(surface, &loops)
}

fn loop_sample(loop_curves: &[NurbsCurve]) -> Result<[f64; 2], KernelRefusal> {
    let pcurve = loop_curves
        .first()
        .ok_or_else(|| "offset carve: an empty trim loop".to_string()).or_refuse(KernelStage::Classify, "carve_empty_loop")?;
    let [t0, t1] = pcurve.domain().or_refuse(KernelStage::Classify, "domain")?;
    let point = pcurve.evaluate(0.5 * (t0 + t1)).or_refuse(KernelStage::Classify, "evaluate")?;
    Ok([point.x, point.y])
}

/// Shoelace area of a side's outer loop, sampled — a report quantity only.
fn parameter_area(side: &[CarvedLoop]) -> f64 {
    let Some(outer) = side.first() else {
        return 0.0;
    };
    let mut points = Vec::new();
    for pcurve in &outer.pcurves {
        let Ok([t0, t1]) = pcurve.domain() else {
            continue;
        };
        for step in 0..32 {
            if let Ok(point) = pcurve.evaluate(t0 + (t1 - t0) * step as f64 / 32.0) {
                points.push([point.x, point.y]);
            }
        }
    }
    if points.len() < 3 {
        return 0.0;
    }
    let mut twice = 0.0;
    for index in 0..points.len() {
        let a = points[index];
        let b = points[(index + 1) % points.len()];
        twice += a[0] * b[1] - b[0] * a[1];
    }
    twice.abs() * 0.5
}
