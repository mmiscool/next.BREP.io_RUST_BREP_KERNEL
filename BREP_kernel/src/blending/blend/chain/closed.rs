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
    // One blend operation: the reports of an earlier one, never consumed, are
    // dropped as it starts (nested calls keep this operation's).
    let _operation = crate::blend::BlendOperation::enter();
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
    // One blend operation: the reports of an earlier one, never consumed, are
    // dropped as it starts (nested calls keep this operation's).
    let _operation = crate::blend::BlendOperation::enter();
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
            // The continuation Newton of one station did not converge from its
            // neighbour at this spacing. Only that typed station failure climbs:
            // the next rung seeds every station from a neighbour half as far
            // away. At the top the march's own refusal is returned verbatim.
            Ok(Rung::StationFailed(failure)) => {
                crate::blend::carve::carve_trace(format_args!(
                    "blend chain: {per_segment} stations per segment: segment {} position {} \
                     (t {:.9}) continued from position {} (t {:.9}, seed {:?}) did not converge: {}",
                    failure.segment, failure.position, failure.t, failure.from_position,
                    failure.from_t, failure.seed_uv, failure.error
                ));
                if per_segment * 2 > CHAIN_CARVE_MAX_PER_SEGMENT {
                    break Err(failure.error);
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
        Ok(Rung::TooCoarse { .. }) | Ok(Rung::StationFailed(_)) => {
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
            // The carve keeps its own contract: a station failure refuses as
            // it did before the plain ladder learned to climb on it.
            Rung::StationFailed(failure) => return Err(failure.error),
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
    /// A station's continuation Newton found no root at this budget
    /// (`march::ChainStationFailure`); a finer rung is a nearer seed.
    StationFailed(ChainStationFailure),
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
///
/// Also returns every interval over `bar`, named by its left sample's index:
/// what local refinement inserts a station into.
fn rail_deviation(
    samples: &[ChainSample],
    cr: &NurbsCurve,
    cs: &NurbsCurve,
    fit_low: f64,
    fit_range: f64,
    per_segment: usize,
    bar: f64,
) -> Result<(f64, Vec<usize>), KernelRefusal> {
    // Each interval runs from a station to the NEXT one along its segment — the
    // next grid position, a station local refinement inserted, or the
    // segment's own end station — in the order the stations take.
    let mut order: Vec<usize> = (0..samples.len())
        .filter(|&index| (0..=per_segment as isize).contains(&samples[index].position))
        .collect();
    order.sort_by(|&a, &b| {
        let (a, b) = (&samples[a], &samples[b]);
        a.segment.cmp(&b.segment).then(a.position.cmp(&b.position)).then(a.offset.total_cmp(&b.offset))
    });
    let next_of: std::collections::HashMap<usize, f64> = order
        .windows(2)
        .filter(|pair| samples[pair[0]].segment == samples[pair[1]].segment)
        .map(|pair| (pair[0], samples[pair[1]].parameter))
        .collect();
    let mut failing = Vec::new();
    let closest = closest_on_rail;
    let mut worst: f64 = 0.0;
    // Each segment's worst halfway contact, for `BREP_BLEND_CARVE_TRACE=1`: where
    // along the chain a rung misses is what tells a junction from a bend.
    let mut per_segment_worst: Vec<(f64, isize, f64, f64, Vec3, Vec3)> = Vec::new();
    for &index in &order {
        let sample = &samples[index];
        let Some([p1, p2]) = sample.midpoint else {
            continue;
        };
        let Some(&next) = next_of.get(&index) else {
            continue;
        };
        let global = (0.5 * (sample.parameter + next)).rem_euclid(1.0);
        let seed = (global - fit_low) / fit_range;
        let reach = (next - sample.parameter).abs() / fit_range;
        let (miss1, miss2) = (closest(cr, p1, seed, reach)?, closest(cs, p2, seed, reach)?);
        worst = worst.max(miss1).max(miss2);
        if miss1.max(miss2) > bar {
            failing.push(index);
        }
        if per_segment_worst.len() <= sample.segment {
            per_segment_worst.resize(
                sample.segment + 1,
                (0.0, 0, 0.0, 0.0, Vec3::default(), Vec3::default()),
            );
        }
        if miss1.max(miss2) >= per_segment_worst[sample.segment].0 {
            per_segment_worst[sample.segment] = (miss1.max(miss2), sample.position, miss1, miss2, p1, p2);
        }
    }
    for (segment, (miss, position, miss1, miss2, p1, p2)) in per_segment_worst.iter().enumerate() {
        crate::blend::carve::carve_trace(format_args!(
            "  rail miss segment {segment}: worst {miss:.6e} after station {position} \
             (cr {miss1:.6e} at ({:.6}, {:.6}, {:.6}), cs {miss2:.6e} at ({:.6}, {:.6}, {:.6}))",
            p1.x, p1.y, p1.z, p2.x, p2.y, p2.z
        ));
    }
    Ok((worst, failing))
}

/// Most dense INTERVALS (4097 samples) a closed chain's support-piece track may
/// hold: the existing ceiling its uniform track is built under, and the one
/// local refinement of that track stays within — not a sample count.
const PIECE_TRACK_CEILING: usize = 4096;

/// Distance from `target` to a fitted rail, by a Newton that stays where it is
/// seeded (`rail_deviation`'s reading, shared with the dense diagnostic).
fn closest_on_rail(curve: &NurbsCurve, target: Vec3, seed: f64, reach: f64) -> Result<f64, KernelRefusal> {
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
}

/// Points of every interval the LOCAL-INSERTION path judges a chain rung's
/// rails on: j/16, j = 1..15 (the halfway point among them).
pub(super) const LOCAL_RAIL_STENCIL: usize = 16;

/// One dense contact pair's miss: each rail's raw reading is checked BEFORE
/// they are joined (`f64::max` alone drops a NaN on either side); `None` when
/// either is unreadable.
fn rail_pair_miss(cr_miss: f64, cs_miss: f64) -> Option<f64> {
    (cr_miss.is_finite() && cs_miss.is_finite()).then(|| cr_miss.max(cs_miss))
}

/// Each solved dense contact's distance to the fitted rails, read with the same
/// closest-point Newton and seeds `rail_deviation` uses, keyed by the
/// interval's left sample: the readable misses, and separately every interval
/// with an unreadable reading on either rail (a non-finite distance, or a foot
/// that could not be evaluated).
#[allow(clippy::too_many_arguments)]
pub(super) fn dense_rail_misses(
    contacts: &crate::blend::chain::DenseContacts,
    samples: &[ChainSample],
    cr: &NurbsCurve,
    cs: &NurbsCurve,
    low: f64,
    range: f64,
    wrap: bool,
) -> Result<(Vec<(usize, f64)>, Vec<usize>), KernelRefusal> {
    let mut readable = Vec::with_capacity(contacts.solved.len());
    let mut unreadable = Vec::new();
    for (left, next, fraction, p1, p2) in &contacts.solved {
        let (from, to) = (samples[*left].parameter, samples[*next].parameter);
        // A closed chain's chord parameter wraps with period 1; an OPEN
        // chain's runs straight through its ends (its last station's may
        // overshoot 1, and wrapping it would read the far end).
        let global = from + fraction * (to - from);
        let global = if wrap { global.rem_euclid(1.0) } else { global };
        let seed = (global - low) / range;
        let reach = (to - from).abs() / range;
        // A closest foot that cannot be evaluated is an unreadable reading of
        // this interval, not a refusal of the rung.
        let cr_miss = closest_on_rail(cr, *p1, seed, reach).unwrap_or(f64::NAN);
        let cs_miss = closest_on_rail(cs, *p2, seed, reach).unwrap_or(f64::NAN);
        match rail_pair_miss(cr_miss, cs_miss) {
            Some(miss) => readable.push((*left, miss)),
            None => unreadable.push(*left),
        }
    }
    Ok((readable, unreadable))
}

/// The local-insertion path's rail verdict: the halfway reading joined with the
/// dense stencil's, FAIL-CLOSED — an interval with a dense contact over `bar`,
/// an unreadable distance, or a contact that was not read (unsolved, or
/// unreadable on either rail), fails; the worst is the largest readable
/// distance. Intervals are named by their left sample.
pub(super) fn combine_rail_verdict(
    halfway: (f64, Vec<usize>),
    dense: &[(usize, f64)],
    unsolved: &[usize],
    bar: f64,
) -> (f64, Vec<usize>) {
    let (mut worst, mut failing) = halfway;
    for &(left, miss) in dense {
        if !miss.is_finite() {
            failing.push(left);
            continue;
        }
        if miss > bar {
            failing.push(left);
        }
        worst = worst.max(miss);
    }
    failing.extend_from_slice(unsolved);
    failing.sort_unstable();
    failing.dedup();
    (worst, failing)
}

/// Whether the construction request runs on an accepted plain fillet rung:
/// always, outside the controls that turn it off on their own thread
/// (`MODEL_REQUEST_OFF`) or that turn the wall verdict off.
/// The construction request's round limit: the existing local-refinement
/// budget, outside the control that sets it to zero on its own thread to
/// force the typed unmet report.
pub(super) fn model_rounds_limit() -> usize {
    REFINE_ROUNDS
}

pub(super) fn model_request_on() -> bool {
    wall_verdict_on()
}


/// Whether the fillet wall verdict runs: always, outside the mutation control
/// that turns it off on its own thread (`WALL_VERDICT_OFF`).
pub(super) fn wall_verdict_on() -> bool {
    true
}





/// Section-angle probes across a fillet wall, as fractions of its `v` span:
/// k/8 for k = 1..7, between (never on) the rails the rail verdict reads.
pub(super) const WALL_SECTION_PROBES: usize = 8;

/// How far `p` stands from the rolling ball's sweep, read at p's OWN
/// characteristic: the stationary point of |p − C(s)|, C the cubic through
/// four consecutive exact ball centres of `nodes` around node `j` (`nodes`
/// uniform in fraction across one interval, its ends the interval's own
/// stations), searched within one node of `j`, minus `radius`.
///
/// The cubic is an APPROXIMATION of the centre curve, not the exact sweep:
/// its error grows with the node spacing and the spine's higher derivatives,
/// and [`wall_probe_reading`] estimates it by a second, overlapping stencil.
///
/// A STATIONARY point, not the nearest centre: past a fold's focal point an
/// envelope point is a MAXIMUM of its distance to the spine, and a global
/// nearest centre would read a carved wall's lens as inside another ball. A
/// centre at the probe's own chain parameter is not used either: the wall's
/// `u` and the stencil's edge parameter name the same section only to first
/// order, and a spine offset ℓ adds √(ρ² + ℓ²) − ρ to the reading.
///
/// `None` — unreadable, which fails its interval — for a non-finite input,
/// fewer than four nodes, a probe at an interval end, or a solve that leaves
/// its window or does not converge.
pub(in crate::blend) fn sweep_residual(p: Vec3, nodes: &[Vec3], j: usize, radius: f64) -> Option<f64> {
    let j = locate_foot(p, nodes, j, radius)?;
    sweep_residual_on(p, nodes, (j - 1).min(nodes.len() - 4), j, radius)
}

/// How many nodes the stencil may follow a probe's foot away from the probe's
/// own node.
const FOOT_TRAVEL: usize = 4;

/// The node whose stencil holds `p`'s stationary point, starting from node
/// `j`: the probe's wall `u` and the centres' edge parameter name the same
/// section only to first order, so the foot can sit more than the one-node
/// window from `j` (the 15- and 20-degree rows' feet stood up to 1.5 nodes,
/// one 3.3, from it on f4ed17782's build, every decline there a window exit).
/// A solve that leaves its window moves the stencil to the node nearest
/// where it left, at most [`FOOT_TRAVEL`] nodes from `j` and never back to a
/// node already tried; `None` when the foot is not found that way (or a
/// solve is unreadable), which fails its interval as before.
fn locate_foot(p: Vec3, nodes: &[Vec3], j: usize, radius: f64) -> Option<usize> {
    let n = nodes.len();
    if n < 4 || j == 0 || j + 1 >= n {
        return None;
    }
    let mut tried: Vec<usize> = Vec::new();
    let mut at = j;
    loop {
        let base = (at - 1).min(n - 4);
        match stencil_solve(p, nodes, base, at, radius) {
            Solve::Read(_) => return Some(at),
            Solve::Unread => return None,
            Solve::Window(s) => {
                tried.push(at);
                let next = (base as f64 + s).round();
                let next = if next.is_finite() { next.clamp(1.0, (n - 2) as f64) as usize } else { at };
                if tried.contains(&next) || next.abs_diff(j) > FOOT_TRAVEL {
                    wall_decline(|| format!(
                        "foot search from node {j} of {n}: tried {tried:?}, next {next} ({})",
                        if tried.contains(&next) { "already tried" } else { "beyond FOOT_TRAVEL" }
                    ));
                    return None;
                }
                at = next;
            }
        }
    }
}

/// [`sweep_residual`] on the cubic through `nodes[base..base + 4]`, the
/// stationary point searched from node `j` within one node of it.
fn sweep_residual_on(p: Vec3, nodes: &[Vec3], base: usize, j: usize, radius: f64) -> Option<f64> {
    match stencil_solve(p, nodes, base, j, radius) {
        Solve::Read(distance) => Some(distance),
        Solve::Window(_) | Solve::Unread => None,
    }
}

/// One stencil's solve: the distance read, or where (in node units from
/// `base`) the search left its one-node window, or unreadable.
enum Solve {
    Read(f64),
    Window(f64),
    Unread,
}

thread_local! {
    /// When Some, every decline inside the declared wall reader appends WHY
    /// (innermost first): set and taken by `declared_wall_probe_why` only, so
    /// every other caller pays one thread-local read per decline and the
    /// reader's arithmetic and outcomes are the same either way.
    static WALL_DECLINES: std::cell::RefCell<Option<Vec<String>>> = const { std::cell::RefCell::new(None) };
}

fn wall_decline(reason: impl FnOnce() -> String) {
    WALL_DECLINES.with(|log| {
        if let Some(lines) = log.borrow_mut().as_mut() {
            lines.push(reason());
        }
    });
}

/// [`declared_wall_probe`] with, when it declines, the chain of declines
/// that made it (innermost first): which tier (fine / coarse), which
/// stencil, and inside the solver the exit (unreadable input, a vanishing
/// slope, a window exit with the overshot `s`, its offset from the window and
/// whether the solver's OWN stationarity predicate already held at the
/// overshot point, a stall, a non-finite distance) or the foot search's
/// tried nodes. Exactly the same arithmetic as `declared_wall_probe`.
pub(in crate::blend) fn declared_wall_probe_why(p: Vec3, fine: &[Vec3], jf: usize, radius: f64) -> Result<(f64, f64), String> {
    let previous = WALL_DECLINES.with(|log| log.replace(Some(Vec::new())));
    let read = declared_wall_probe(p, fine, jf, radius);
    let lines = WALL_DECLINES.with(|log| log.replace(previous)).unwrap_or_default();
    read.ok_or_else(|| if lines.is_empty() { "declined (no inner cause recorded)".to_string() } else { lines.join(" <- ") })
}

fn stencil_solve(p: Vec3, nodes: &[Vec3], base: usize, j: usize, radius: f64) -> Solve {
    let finite = |v: Vec3| v.x.is_finite() && v.y.is_finite() && v.z.is_finite();
    let n = nodes.len();
    if n < 4 || j == 0 || j + 1 >= n || base + 4 > n || !radius.is_finite() || !finite(p) || !nodes.iter().all(|&c| finite(c)) {
        wall_decline(|| format!("stencil {n} nodes base {base} j {j}: unreadable input"));
        return Solve::Unread;
    }
    // Newton's divided differences on 0, 1, 2, 3, s in node units from `base`.
    let [p0, p1, p2, p3] = [nodes[base], nodes[base + 1], nodes[base + 2], nodes[base + 3]];
    let d1 = p1.sub(p0);
    let d2 = p2.sub(p1.scale(2.0)).add(p0).scale(0.5);
    let d3 = p3.sub(p2.scale(3.0)).add(p1.scale(3.0)).sub(p0).scale(1.0 / 6.0);
    let curve = |s: f64| -> (Vec3, Vec3, Vec3) {
        let c = p0.add(d1.scale(s)).add(d2.scale(s * (s - 1.0))).add(d3.scale(s * (s - 1.0) * (s - 2.0)));
        let c1 = d1.add(d2.scale(2.0 * s - 1.0)).add(d3.scale(3.0 * s * s - 6.0 * s + 2.0));
        let c2 = d2.scale(2.0).add(d3.scale(6.0 * s - 6.0));
        (c, c1, c2)
    };
    let start = j as f64 - base as f64;
    let mut s = start;
    let mut converged = false;
    for _ in 0..50 {
        let (c, c1, c2) = curve(s);
        let w = c.sub(p);
        let (g, slope) = (w.dot(c1), c1.dot(c1) + w.dot(c2));
        if !(g.is_finite() && slope.is_finite()) || slope == 0.0 {
            wall_decline(|| format!("stencil {n} nodes base {base} j {j}: slope {slope:e} g {g:e} at s {s}"));
            return Solve::Unread;
        }
        // Converged at THIS s once the stationarity condition holds to its own
        // rounding: g = (C − p)·C′ carries an absolute error of a few
        // ε·(|p| + |C|)·|C′|, so at a node spacing of 1e-4 the Newton step
        // cannot fall below ~1e-11 node units and the relative step test
        // alone never fires — a line-for-line model of this reader read 87 %
        // to 97 % of the NX and skew walls' fine-stencil probes unreadable
        // that way (its Newton run past the stop read |g| / (ε·(|p| + |C|)·|C′|)
        // at most 0.91 there; 16 keeps a margin). Tested before the step, so
        // the point read is the point whose stationarity was checked; its
        // distance is off the true stationary one by about
        // ½·|g|·δs/|C − p| ≤ 8ε·(|p| + |C|)·|C′|/ρ for δs within the
        // one-node window, far below any bar.
        let stationary = g.abs() <= 16.0 * f64::EPSILON * (p.length() + c.length()) * c1.length();
        if stationary {
            converged = true;
            break;
        }
        let step = g / slope;
        let before = s;
        s -= step;
        if !((s - start).abs() <= 1.0) {
            wall_decline(|| {
                // The solver's own stationarity predicate, read at the
                // overshot point (diagnostic only: nothing decides on it).
                let (c, c1, _) = curve(s);
                let g_after = c.sub(p).dot(c1);
                let stationary_after = g_after.abs() <= 16.0 * f64::EPSILON * (p.length() + c.length()) * c1.length();
                format!(
                    "stencil {n} nodes base {base} j {j}: window exit s {before} -> {s} (start {start}, |s - start| - 1 = {:e}), \
                     g {g:e} -> {g_after:e}, stationary at the overshot s: {stationary_after}",
                    (s - start).abs() - 1.0
                )
            });
            return if s.is_finite() { Solve::Window(s) } else { Solve::Unread };
        }
        if step.abs() <= 1e-14 * (1.0 + s.abs()) {
            converged = true;
            break;
        }
    }
    if !converged {
        wall_decline(|| format!("stencil {n} nodes base {base} j {j}: no convergence in 50 steps, s {s}"));
        return Solve::Unread;
    }
    let distance = curve(s).0.sub(p).length() - radius;
    if distance.is_finite() {
        Solve::Read(distance)
    } else {
        wall_decline(|| format!("stencil {n} nodes base {base} j {j}: non-finite distance at s {s}"));
        Solve::Unread
    }
}

/// One wall probe read on TWO overlapping four-centre stencils around node
/// `j` (the one [`sweep_residual`] uses and its neighbour): `(worst, spread)`
/// — the larger magnitude, which keeps ONE stencil's under-read from passing,
/// and how far apart the two readings are. Both are PROXY readings: when both
/// stencils under-read alike (a symmetric node set about a foot near the
/// focal distance) the larger still under-reads and the spread reads zero,
/// so neither number is an error bound. `None` when either stencil is
/// unreadable.
pub(in crate::blend) fn wall_probe_reading(p: Vec3, nodes: &[Vec3], j: usize, radius: f64) -> Option<(f64, f64)> {
    let n = nodes.len();
    if n < 5 || j == 0 || j + 1 >= n {
        return None;
    }
    let j = locate_foot(p, nodes, j, radius)?;
    let first = (j - 1).min(n - 4);
    let second = if j >= 2 { (j - 2).min(n - 4) } else { j.min(n - 4) };
    let second = if second == first { if first >= 1 { first - 1 } else { first + 1 } } else { second };
    let a = sweep_residual_on(p, nodes, first, j, radius)?;
    let b = sweep_residual_on(p, nodes, second, j, radius)?;
    Some((a.abs().max(b.abs()), (a - b).abs()))
}

/// The DECLARED wall reading at one probe, the one rule both wall tiers
/// decide on: `(reading, uncertainty)`, declared = reading + uncertainty.
/// `fine` are the interval's centres at twice the probe density, `jf` the
/// probe's (even) node among them; the coarse set is every other node. The
/// reading is the larger fine two-stencil magnitude; the uncertainty is the
/// larger of the fine stencils' spread and how far the fine reading moved
/// from the coarse one — the proxy's error estimated from its own change with
/// node density, which two stencils agreeing at one density cannot show (two
/// symmetric stencils that both under-read read a spread of zero). An
/// ESTIMATE, not a bound: a proxy whose error does not shrink with density
/// would still hide. `None` when any reading is unreadable.
pub(in crate::blend) fn declared_wall_probe(p: Vec3, fine: &[Vec3], jf: usize, radius: f64) -> Option<(f64, f64)> {
    if jf % 2 != 0 || fine.len() < 9 || fine.len() % 2 == 0 {
        wall_decline(|| format!("declared reader: node {jf} of {} not an even node of an odd fine set", fine.len()));
        return None;
    }
    let Some((reading, spread)) = wall_probe_reading(p, fine, jf, radius) else {
        wall_decline(|| format!("FINE two-stencil reading at node {jf} of {}", fine.len()));
        return None;
    };
    let Some(at_fine) = sweep_residual(p, fine, jf, radius) else {
        wall_decline(|| format!("FINE sweep at node {jf} of {}", fine.len()));
        return None;
    };
    let coarse: Vec<Vec3> = fine.iter().step_by(2).copied().collect();
    let Some(at_coarse) = sweep_residual(p, &coarse, jf / 2, radius) else {
        wall_decline(|| format!("COARSE sweep at node {} of {}", jf / 2, coarse.len()));
        return None;
    };
    let uncertainty = spread.max((at_fine - at_coarse).abs());
    uncertainty.is_finite().then_some((reading, uncertainty))
}

/// The even fine node whose position along the CENTRE PATH (cumulative
/// chord of `fine`) is nearest `fraction` of it, inside 2..=n-3; `None` when
/// the path has no finite positive length.
///
/// A wall probe sits at a fraction of its interval in the WALL's parameter,
/// which is chord-like; the exact centres sit at uniform fractions of the
/// edge's NATIVE parameter. On a regular edge the two agree to first order and
/// the probe's foot lies within a node or two of its own node. At a
/// STATIONARY section (the edge's derivative vanishes there) the centre moves
/// like t^2 in the native parameter, so a probe at chord fraction f has its foot
/// near native fraction sqrt(f): the probe at node 2 of 32 (f = 1/16) has its
/// foot near node 8, past `FOOT_TRAVEL`, and the reader declines though the
/// centre locus is perfectly regular there (zero_tangent_sites' stationary
/// stadium and tombstone rims refused that way with every reading under the
/// bar).
pub(in crate::blend) fn centre_path_node(fine: &[Vec3], fraction: f64) -> Option<usize> {
    let n = fine.len();
    if n < 7 || !(fraction.is_finite()) {
        return None;
    }
    let mut cumulative = Vec::with_capacity(n);
    cumulative.push(0.0_f64);
    for pair in fine.windows(2) {
        let step = pair[1].sub(pair[0]).length();
        cumulative.push(cumulative.last().copied().unwrap_or(0.0) + step);
    }
    let total = *cumulative.last()?;
    if !(total.is_finite() && total > 0.0) {
        return None;
    }
    let target = fraction.clamp(0.0, 1.0) * total;
    let nearest = (0..n).min_by(|&a, &b| (cumulative[a] - target).abs().total_cmp(&(cumulative[b] - target).abs()))?;
    let even = 2 * ((nearest as f64 / 2.0).round() as usize);
    Some(even.clamp(2, (n - 3) & !1))
}

/// [`declared_wall_probe_why`] from the probe's own node `jf`; when that
/// declines, once more from the node the probe's `fraction` reaches along the
/// centre path ([`centre_path_node`]), when that is another node. The same
/// reader, bars and fail-closed rule decide; a probe it still cannot read is
/// unread (both declines named). A probe the first read reads is read exactly
/// as before.
pub(in crate::blend) fn declared_wall_probe_located_why(p: Vec3, fine: &[Vec3], jf: usize, fraction: f64, radius: f64) -> Result<(f64, f64), String> {
    let first = match declared_wall_probe_why(p, fine, jf, radius) {
        Ok(read) => return Ok(read),
        Err(why) => why,
    };
    match centre_path_node(fine, fraction) {
        Some(located) if located != jf => declared_wall_probe_why(p, fine, located, radius)
            .map_err(|again| format!("{first} -- and from centre-path node {located}: {again}")),
        _ => Err(first),
    }
}

/// [`declared_wall_probe_located_why`] as an `Option`.
pub(in crate::blend) fn declared_wall_probe_located(p: Vec3, fine: &[Vec3], jf: usize, fraction: f64, radius: f64) -> Option<(f64, f64)> {
    match declared_wall_probe(p, fine, jf, radius) {
        Some(read) => Some(read),
        None => match centre_path_node(fine, fraction) {
            Some(located) if located != jf => declared_wall_probe(p, fine, located, radius),
            _ => None,
        },
    }
}

/// [`declared_wall_probe_why`] on centres at NON-UNIFORM native abscissae
/// `ts` (one per node, strictly increasing): the same two-tier rule, the
/// same two overlapping four-centre stencils, the same foot search
/// (`FOOT_TRAVEL`), the same 16·ε stationarity predicate and the same
/// window (one node either side of the start node), with each cubic the
/// Newton interpolant on the nodes' OWN abscissae instead of the implicit
/// 0, 1, 2, 3. For a junction of two intervals of unequal native width (local
/// insertions make them so): the uniform reader would place the two halves'
/// nodes at one spacing and distort the centre locus. The coarse tier is
/// every other node at every other abscissa. A caller whose abscissae are
/// uniform uses the uniform reader, bit for bit.
pub(in crate::blend) fn declared_wall_probe_at_why(p: Vec3, fine: &[Vec3], ts: &[f64], jf: usize, radius: f64) -> Result<(f64, f64), String> {
    if jf % 2 != 0
        || fine.len() < 9
        || fine.len() % 2 == 0
        || ts.len() != fine.len()
        || !ts.iter().all(|t| t.is_finite())
        || !ts.windows(2).all(|pair| pair[1] > pair[0])
    {
        return Err(format!("native-abscissa reader: node {jf} of {} with {} finite, strictly increasing abscissae required", fine.len(), ts.len()));
    }
    let previous = WALL_DECLINES.with(|log| log.replace(Some(Vec::new())));
    let read = (|| -> Option<(f64, f64)> {
        let (reading, spread) = wall_probe_reading_at(p, fine, ts, jf, radius).or_else(|| {
            wall_decline(|| format!("FINE two-stencil reading (native abscissae) at node {jf} of {}", fine.len()));
            None
        })?;
        let at_fine = sweep_residual_at(p, fine, ts, jf, radius).or_else(|| {
            wall_decline(|| format!("FINE sweep (native abscissae) at node {jf} of {}", fine.len()));
            None
        })?;
        let coarse: Vec<Vec3> = fine.iter().step_by(2).copied().collect();
        let coarse_ts: Vec<f64> = ts.iter().step_by(2).copied().collect();
        let at_coarse = sweep_residual_at(p, &coarse, &coarse_ts, jf / 2, radius).or_else(|| {
            wall_decline(|| format!("COARSE sweep (native abscissae) at node {} of {}", jf / 2, coarse.len()));
            None
        })?;
        let uncertainty = spread.max((at_fine - at_coarse).abs());
        uncertainty.is_finite().then_some((reading, uncertainty))
    })();
    let lines = WALL_DECLINES.with(|log| log.replace(previous)).unwrap_or_default();
    read.ok_or_else(|| if lines.is_empty() { "declined (no inner cause recorded)".to_string() } else { lines.join(" <- ") })
}

fn wall_probe_reading_at(p: Vec3, nodes: &[Vec3], ts: &[f64], j: usize, radius: f64) -> Option<(f64, f64)> {
    let n = nodes.len();
    if n < 5 || j == 0 || j + 1 >= n {
        return None;
    }
    let j = locate_foot_at(p, nodes, ts, j, radius)?;
    let first = (j - 1).min(n - 4);
    let second = if j >= 2 { (j - 2).min(n - 4) } else { j.min(n - 4) };
    let second = if second == first { if first >= 1 { first - 1 } else { first + 1 } } else { second };
    let a = match stencil_solve_at(p, nodes, ts, first, j, radius) { Solve::Read(distance) => distance, _ => return None };
    let b = match stencil_solve_at(p, nodes, ts, second, j, radius) { Solve::Read(distance) => distance, _ => return None };
    Some((a.abs().max(b.abs()), (a - b).abs()))
}

fn sweep_residual_at(p: Vec3, nodes: &[Vec3], ts: &[f64], j: usize, radius: f64) -> Option<f64> {
    let j = locate_foot_at(p, nodes, ts, j, radius)?;
    match stencil_solve_at(p, nodes, ts, (j - 1).min(nodes.len() - 4), j, radius) {
        Solve::Read(distance) => Some(distance),
        Solve::Window(_) | Solve::Unread => None,
    }
}

/// [`locate_foot`] on native abscissae: a window exit moves the stencil to
/// the node whose abscissa is nearest the exit, within `FOOT_TRAVEL` nodes
/// and never back to a node already tried.
fn locate_foot_at(p: Vec3, nodes: &[Vec3], ts: &[f64], j: usize, radius: f64) -> Option<usize> {
    let n = nodes.len();
    if n < 4 || j == 0 || j + 1 >= n {
        return None;
    }
    let mut tried: Vec<usize> = Vec::new();
    let mut at = j;
    loop {
        let base = (at - 1).min(n - 4);
        match stencil_solve_at(p, nodes, ts, base, at, radius) {
            Solve::Read(_) => return Some(at),
            Solve::Unread => return None,
            Solve::Window(t) => {
                tried.push(at);
                let nearest = (0..n).min_by(|&a, &b| (ts[a] - t).abs().total_cmp(&(ts[b] - t).abs())).unwrap_or(at);
                let next = if t.is_finite() { nearest.clamp(1, n - 2) } else { at };
                if tried.contains(&next) || next.abs_diff(j) > FOOT_TRAVEL {
                    wall_decline(|| format!(
                        "foot search (native abscissae) from node {j} of {n}: tried {tried:?}, next {next} ({})",
                        if tried.contains(&next) { "already tried" } else { "beyond FOOT_TRAVEL" }
                    ));
                    return None;
                }
                at = next;
            }
        }
    }
}

/// [`stencil_solve`] on the four nodes `base..base + 4` at their native
/// abscissae: the Newton (divided-difference) cubic in `t`, searched from
/// node `j`'s abscissa within one node either side; `Window` carries the
/// abscissa it left at.
fn stencil_solve_at(p: Vec3, nodes: &[Vec3], ts: &[f64], base: usize, j: usize, radius: f64) -> Solve {
    let finite = |v: Vec3| v.x.is_finite() && v.y.is_finite() && v.z.is_finite();
    let n = nodes.len();
    if n < 4 || j == 0 || j + 1 >= n || base + 4 > n || ts.len() != n || !radius.is_finite() || !finite(p) || !nodes.iter().all(|&c| finite(c)) {
        wall_decline(|| format!("native stencil {n} nodes base {base} j {j}: unreadable input"));
        return Solve::Unread;
    }
    let [p0, p1, p2, p3] = [nodes[base], nodes[base + 1], nodes[base + 2], nodes[base + 3]];
    let [t0, t1, t2, t3] = [ts[base], ts[base + 1], ts[base + 2], ts[base + 3]];
    let d1 = p1.sub(p0).scale(1.0 / (t1 - t0));
    let d12 = p2.sub(p1).scale(1.0 / (t2 - t1));
    let d23 = p3.sub(p2).scale(1.0 / (t3 - t2));
    let d2 = d12.sub(d1).scale(1.0 / (t2 - t0));
    let d123 = d23.sub(d12).scale(1.0 / (t3 - t1));
    let d3 = d123.sub(d2).scale(1.0 / (t3 - t0));
    let curve = |t: f64| -> (Vec3, Vec3, Vec3) {
        let (a, b, c) = (t - t0, t - t1, t - t2);
        let point = p0.add(d1.scale(a)).add(d2.scale(a * b)).add(d3.scale(a * b * c));
        let first = d1.add(d2.scale(a + b)).add(d3.scale(b * c + a * c + a * b));
        let second = d2.scale(2.0).add(d3.scale(2.0 * (a + b + c)));
        (point, first, second)
    };
    let (low, high) = (ts[j - 1], ts[j + 1]);
    let start = ts[j];
    let mut t = start;
    let mut converged = false;
    for _ in 0..50 {
        let (c, c1, c2) = curve(t);
        let w = c.sub(p);
        let (g, slope) = (w.dot(c1), c1.dot(c1) + w.dot(c2));
        if !(g.is_finite() && slope.is_finite()) || slope == 0.0 {
            wall_decline(|| format!("native stencil {n} nodes base {base} j {j}: slope {slope:e} g {g:e} at t {t}"));
            return Solve::Unread;
        }
        if g.abs() <= 16.0 * f64::EPSILON * (p.length() + c.length()) * c1.length() {
            converged = true;
            break;
        }
        let step = g / slope;
        let before = t;
        t -= step;
        if !(t >= low && t <= high) {
            wall_decline(|| format!("native stencil {n} nodes base {base} j {j}: window exit t {before} -> {t} (window [{low}, {high}])"));
            return if t.is_finite() { Solve::Window(t) } else { Solve::Unread };
        }
        // The uniform reader's stop, 1e-14 · (1 + |s|) in node units from
        // the stencil's base, read in LOCAL node units here (one node = half
        // the window), so it does not depend on where the native t's origin
        // is or on its scale.
        let node = 0.5 * (high - low);
        if (step / node).abs() <= 1e-14 * (1.0 + ((t - t0) / node).abs()) {
            converged = true;
            break;
        }
    }
    if !converged {
        wall_decline(|| format!("native stencil {n} nodes base {base} j {j}: no convergence in 50 steps, t {t}"));
        return Solve::Unread;
    }
    let distance = curve(t).0.sub(p).length() - radius;
    if distance.is_finite() {
        Solve::Read(distance)
    } else {
        wall_decline(|| format!("native stencil {n} nodes base {base} j {j}: non-finite distance at t {t}"));
        Solve::Unread
    }
}

/// The intervals the construction request refines (and, when it cannot,
/// reports): every interval whose DECLARED wall reading — as
/// `wall_interior_misses` returns it, `(left, declared, reading,
/// uncertainty)` — exceeds `model`. The one decision the request makes.
pub(in crate::blend) fn wall_model_missing(readings: &[(usize, f64, f64, f64)], model: f64) -> Vec<usize> {
    readings.iter().filter(|&&(_, declared, _, _)| !(declared <= model)).map(|&(left, _, _, _)| left).collect()
}

/// Every fillet-wall interval's worst distance from the rolling ball's sweep,
/// read on the BUILT wall at the even fractions of the centre stencil
/// j/`subdivisions` (the centres at twice the probe density) and section
/// angles k/`probes` ([`declared_wall_probe`]), keyed by the interval's left
/// sample: each readable interval's largest DECLARED reading, largest reading
/// and largest uncertainty, and separately
/// every interval with an unreadable reading or a contact the stencil did not
/// solve. The wall shares the rails' `u` (one interpolation parameter for its
/// three rows).
#[allow(clippy::too_many_arguments)]
pub(super) fn wall_interior_misses(
    contacts: &crate::blend::chain::DenseContacts,
    samples: &[ChainSample],
    surface: &crate::NurbsSurface,
    low: f64,
    range: f64,
    radius: f64,
    subdivisions: usize,
    probes: usize,
    wrap: bool,
) -> (Vec<(usize, f64, f64, f64)>, Vec<usize>) {
    use std::collections::BTreeMap;
    // Each interval's centres at fractions 0, 1/k, …, 1: its stations at the
    // ends, the stencil's solved balls between.
    let mut nodes: BTreeMap<usize, (usize, Vec<Option<Vec3>>)> = BTreeMap::new();
    for (index, (left, next, fraction, _, _)) in contacts.solved.iter().enumerate() {
        let entry = nodes.entry(*left).or_insert_with(|| {
            let mut row = vec![None; subdivisions + 1];
            row[0] = Some(samples[*left].station.center);
            row[subdivisions] = Some(samples[*next].station.center);
            (*next, row)
        });
        let step = (fraction * subdivisions as f64).round() as usize;
        if (1..subdivisions).contains(&step) {
            entry.1[step] = contacts.centers.get(index).copied().flatten();
        }
    }
    let mut readable = Vec::new();
    let mut unreadable: Vec<usize> = contacts.unsolved_at.clone();
    for (&left, (next, row)) in &nodes {
        let Some(centres) = row.iter().copied().collect::<Option<Vec<Vec3>>>() else {
            unreadable.push(left);
            continue;
        };
        let (from, to) = (samples[left].parameter, samples[*next].parameter);
        let (mut declared, mut worst, mut spread) = (0.0_f64, 0.0_f64, 0.0_f64);
        let mut read = true;
        'probes: for step in (2..subdivisions).step_by(2) {
            let global = from + (step as f64 / subdivisions as f64) * (to - from);
            let global = if wrap { global.rem_euclid(1.0) } else { global };
            let u = (global - low) / range;
            for k in 1..probes {
                let v = k as f64 / probes as f64;
                let reading = surface.evaluate(u, v).ok().and_then(|p| {
                    declared_wall_probe_located(p, &centres, step, step as f64 / subdivisions as f64, radius)
                });
                match reading {
                    Some((distance, gap)) => {
                        declared = declared.max(distance + gap);
                        worst = worst.max(distance);
                        spread = spread.max(gap);
                    }
                    None => {
                        read = false;
                        break 'probes;
                    }
                }
            }
        }
        if read {
            readable.push((left, declared, worst, spread));
        } else {
            unreadable.push(left);
        }
    }
    unreadable.sort_unstable();
    unreadable.dedup();
    (readable, unreadable)
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
    let samples = match march_chain_typed(segments, radius, false, per_segment, fold) {
        Ok(samples) => samples,
        Err(ChainMarchError::Station(failure)) => return Ok(Rung::StationFailed(failure)),
        // Only local refinement reports incoherence; the march never does, so
        // this is its refusal exactly as `march_chain` would return it.
        Err(ChainMarchError::Incoherent(failure)) => return Err(failure.error),
        Err(ChainMarchError::Other(error)) => return Err(error),
    };
    CHAIN_DEEPEST_ROUND.with(|deepest| deepest.set(0));
    build_closed_chain_from_samples(solid, segments, radius, chamfer, name, per_segment, fold, bar, samples, 0)
}


thread_local! {
    /// The deepest local round any rung of the current ladder step attempted
    /// (reset as each rung's march is built from), so a rejected refinement
    /// reports the rounds it actually spent.
    static CHAIN_DEEPEST_ROUND: std::cell::Cell<usize> = const { std::cell::Cell::new(0) };
}

/// Fit, measure and build one rung from its stations. A plain (non-carve)
/// rung whose fitted rails miss the halfway contacts by more than `bar`
/// first REFINES LOCALLY: one continuation station into each failing interval
/// (`march::insert_chain_stations`), up to [`REFINE_ROUNDS`] rounds and never
/// past [`CHAIN_CARVE_MAX_PER_SEGMENT`] stations in a segment, then measures
/// again. Only a rung whose local rounds are spent is too coarse, and the
/// ladder climbs as it always has. A rung that passes at once is built from
/// exactly the stations it was built from before.
#[allow(clippy::too_many_arguments)]
fn build_closed_chain_from_samples(
    solid: &BrepSolid,
    segments: &[ChainSegment<'_>],
    radius: f64,
    chamfer: bool,
    name: Option<&str>,
    per_segment: usize,
    fold: FoldPolicy,
    bar: f64,
    mut samples: Vec<ChainSample>,
    round: usize,
) -> Result<Rung, KernelRefusal> {
    CHAIN_DEEPEST_ROUND.with(|deepest| deepest.set(deepest.get().max(round)));

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
    let (halfway, halfway_failing) = rail_deviation(&samples, &cr, &cs, low, range, per_segment, bar)?;
    // A rung that local insertion has refined is judged on the dense stencil of
    // exact contacts, not the halfway point alone (the NX collar's rung 24 read
    // 1.943844e-6 halfway and 2.009538e-6 at 7/16 against 2e-6). A rung that
    // passes without insertion is read exactly as it always was.
    // A FILLET wall is also judged between its rails, on every rung (carve
    // included): the rails can pass while the wall's interior — its weighted
    // apex row interpolated between stations — stands off the rolling ball
    // (the 20-degree seam-split row's accepted wall read 2.4335e-6 on a 16 x 9
    // probe grid and 2.4430e-6 at its worst interval's midpoint, mid-section,
    // against 2e-6). A chamfer is not a rolling ball and is not judged
    // against one. One stencil of exact contacts serves rails and wall.
    let stencil = if round > 0 {
        Some(crate::blend::chain::chain_dense_contacts(segments, radius, per_segment, &samples, LOCAL_RAIL_STENCIL)?)
    } else {
        None
    };
    // The wall's own centres, at twice the rail stencil's density: its probes
    // sit at the rail stencil's fractions, and each is read at both densities
    // (`declared_wall_probe`). The rails keep their own stencil, read exactly
    // as before.
    let wall_stencil = if !chamfer && wall_verdict_on() {
        Some(crate::blend::chain::chain_dense_contacts(segments, radius, per_segment, &samples, 2 * LOCAL_RAIL_STENCIL)?)
    } else {
        None
    };
    let (mut deviation, mut failing, _stencil_reads) = if round > 0 {
        let contacts = stencil.as_ref().expect("the local path solves the stencil");
        let (dense, unreadable) = dense_rail_misses(contacts, &samples, &cr, &cs, low, range, true)?;
        let reads = dense.len() + unreadable.len() + contacts.unsolved;
        let failed_to_read: Vec<usize> = contacts.unsolved_at.iter().chain(&unreadable).copied().collect();
        let verdict = combine_rail_verdict((halfway, halfway_failing), &dense, &failed_to_read, bar);
        crate::blend::carve::carve_trace(format_args!(
            "blend chain local rails: rung {per_segment} round {round}: stencil verdict {:.9e} (halfway {halfway:.9e}), \
             {} failing interval(s), {} unsolved contact(s)",
            verdict.0, verdict.1.len(), contacts.unsolved
        ));
        (verdict.0, verdict.1, reads)
    } else {
        (halfway, halfway_failing, 0)
    };
    // The wall between the rails, joined FAIL-CLOSED: an interval whose wall
    // reads over `bar`, or cannot be read, fails exactly as a rail interval
    // does, and the ladder (local insertion, climb, the existing refusal)
    // decides as it always has. Nothing else about the rung changes: a wall
    // inside `bar` adds reads, never stations.
    let mut _wall_worst = 0.0_f64;
    let mut _wall_spread = 0.0_f64;
    let mut wall_readings: Vec<(usize, f64, f64, f64)> = Vec::new();
    if let Some(contacts) = wall_stencil.as_ref() {
        let (wall, unreadable) =
            wall_interior_misses(contacts, &samples, &surface, low, range, radius, 2 * LOCAL_RAIL_STENCIL, WALL_SECTION_PROBES, true);
        wall_readings = wall.clone();
        // Acceptance on the DECLARED reading against the unchanged bar.
        for &(left, declared, _, uncertainty) in &wall {
            if declared > bar {
                failing.push(left);
            }
            _wall_worst = _wall_worst.max(declared);
            _wall_spread = _wall_spread.max(uncertainty);
        }
        failing.extend_from_slice(&unreadable);
        failing.sort_unstable();
        failing.dedup();
        deviation = deviation.max(_wall_worst);
        crate::blend::carve::carve_trace(format_args!(
            "blend chain wall: rung {per_segment} round {round}: wall interior worst {_wall_worst:.9e} over {} \
             interval(s), {} unreadable, bar {bar:.3e}",
            wall.len(),
            unreadable.len()
        ));
    }
    if deviation > bar || !failing.is_empty() {
        let in_segment = |segment: usize| {
            samples
                .iter()
                .filter(|sample| sample.segment == segment && (0..per_segment as isize).contains(&sample.position))
                .count()
        };
        let within_ceiling = (0..segments.len()).all(|segment| {
            in_segment(segment) + failing.iter().filter(|&&index| samples[index].segment == segment).count()
                <= CHAIN_CARVE_MAX_PER_SEGMENT
        });
        crate::blend::carve::carve_trace(format_args!(
            "blend chain local rails: rung {per_segment} round {round} stations {} failing {} \
             worst {deviation:.9e} bar {bar:.9e} ceiling {}",
            samples.len(),
            failing.len(),
            if within_ceiling { "ok" } else { "reached" }
        ));
        if fold != FoldPolicy::Refuse || round >= REFINE_ROUNDS || !within_ceiling {
            return Ok(Rung::TooCoarse { deviation });
        }
        // The insertion is the next round's attempt, whatever it returns.
        CHAIN_DEEPEST_ROUND.with(|deepest| deepest.set(deepest.get().max(round + 1)));
        match insert_chain_stations(segments, radius, per_segment, &mut samples, &failing) {
            Ok(()) => {}
            Err(ChainMarchError::Station(failure)) => return Ok(Rung::StationFailed(failure)),
            // An inserted station that does not reach its retained right
            // neighbour's branch: this rung cannot be refined coherently, so it
            // is as coarse as its measured rails say, and the ladder decides.
            Err(ChainMarchError::Incoherent(failure)) => {
                crate::blend::carve::carve_trace(format_args!(
                    "blend chain local rails: rung {per_segment} round {round}: segment {} station \
                     at t {:.9} does not reach its retained right neighbour: {}",
                    failure.segment, failure.t, failure.error
                ));
                return Ok(Rung::TooCoarse { deviation });
            }
            Err(ChainMarchError::Other(error)) => return Err(error),
        }
        assign_chain_parameters(&mut samples, segments.len(), per_segment, false);
        return build_closed_chain_from_samples(
            solid, segments, radius, chamfer, name, per_segment, fold, bar, samples, round + 1,
        );
    }
    crate::blend::carve::carve_trace(format_args!(
        "blend chain: {per_segment} stations per segment leaves the rails {deviation:.6e} \
         from the rolling ball, inside {bar:.3e}"
    ));
    // The CONSTRUCTION request, plain fillet rungs only: an ACCEPTED wall is
    // refined toward `KernelTolerances::model` — the identity tolerance, the
    // same 1e-7 the pcurve construction floor refines to under its looser
    // acceptance — locally, in exactly the intervals whose DECLARED reading
    // (`declared_wall_probe`: the proxy reading plus its uncertainty from
    // stencil spread AND change with node density) is over it: one rule for
    // the acceptance verdict, this request and its report. An estimate with
    // its uncertainty, NOT a proven bound.
    // Acceptance is unchanged: the refined rung must pass it again, and every
    // retained station, branch guard and ceiling holds. A wall already inside
    // `model` everywhere inserts nothing and builds exactly as it did. When
    // refinement cannot run or does not build, THIS accepted rung stands and
    // says so, typed (`WallModelReport`); never silently.
    let model = crate::KernelTolerances::for_solid(solid, 1e-7).model;
    let stations_now = (0..segments.len())
        .map(|segment| {
            samples
                .iter()
                .filter(|sample| sample.segment == segment && (0..per_segment as isize).contains(&sample.position))
                .count()
        })
        .max()
        .unwrap_or(0);
    let mut unmet: Option<(f64, crate::BudgetReason)> = None;
    // What the request found short, named in its report: the detail, which
    // component the residual is the reading of, and how many intervals it
    // could not read (then the true worst is unknown).
    let mut request_detail: Option<String> = None;
    let mut request_component = crate::MeasuredComponent::Wall;
    let mut request_unread = 0usize;
    if !chamfer && fold == FoldPolicy::Refuse && model_request_on() {
        let wall_missing = wall_model_missing(&wall_readings, model);
        // The RAILS against the same request: the shipped support curves ARE
        // these rails (`chain_surgery` splits `cr`/`cs` into the support
        // edges), so a rail standing off the exact construction ships that
        // miss on its carrier. Read against the exact contacts the wall
        // reading already solved (j/32 of every interval); an interval whose
        // rail reads over `model`, or cannot be read, is short exactly as a
        // wall interval is. Acceptance (`bar`) is unchanged.
        let (rail_dense, rail_unread) = match wall_stencil.as_ref() {
            Some(contacts) => dense_rail_misses(contacts, &samples, &cr, &cs, low, range, true)?,
            None => (Vec::new(), Vec::new()),
        };
        // Per contact, keyed by the interval's left sample: distinct intervals.
        let rail_over: std::collections::BTreeSet<usize> =
            rail_dense.iter().filter(|&&(_, miss)| !(miss <= model)).map(|&(left, _)| left).collect();
        let rail_unread_intervals: std::collections::BTreeSet<usize> = rail_unread.iter().copied().collect();
        let rail_missing: Vec<usize> = rail_over.union(&rail_unread_intervals).copied().collect();
        let mut missing: Vec<usize> = wall_missing.iter().chain(&rail_missing).copied().collect();
        missing.sort_unstable();
        missing.dedup();
        if !missing.is_empty() {
            // The same declared reading, over the intervals short.
            let wall_short = wall_readings
                .iter()
                .filter(|reading| wall_missing.contains(&reading.0))
                .map(|&(_, declared, _, _)| declared)
                .fold(0.0_f64, f64::max);
            let rail_short = rail_dense.iter().filter(|&&(_, miss)| !(miss <= model)).map(|&(_, miss)| miss).fold(0.0_f64, f64::max);
            let short = wall_short.max(rail_short);
            request_component = if rail_short > wall_short { crate::MeasuredComponent::Rails } else { crate::MeasuredComponent::Wall };
            request_unread = rail_unread_intervals.len();
            request_detail = Some(format!(
                "short of {model:.3e}: rails {rail_short:.3e} off the exact contacts over {} interval(s) ({} unread), \
                 wall {wall_short:.3e} over {} interval(s)",
                rail_over.len(),
                rail_unread_intervals.len(),
                wall_missing.len()
            ));
            let within_ceiling = (0..segments.len()).all(|segment| {
                let present = samples
                    .iter()
                    .filter(|sample| sample.segment == segment && (0..per_segment as isize).contains(&sample.position))
                    .count();
                present + missing.iter().filter(|&&index| samples[index].segment == segment).count()
                    <= CHAIN_CARVE_MAX_PER_SEGMENT
            });
            crate::blend::carve::carve_trace(format_args!(
                "blend chain wall model: rung {per_segment} round {round}: {} interval(s) short of {model:.3e} \
                 (worst reading + spread {short:.3e}), ceiling {}",
                missing.len(),
                if within_ceiling { "ok" } else { "reached" }
            ));
            unmet = if round >= model_rounds_limit() {
                Some((short, crate::BudgetReason::RoundsSpent))
            } else if !within_ceiling {
                Some((short, crate::BudgetReason::StationCeiling))
            } else {
                let mut refined = samples.clone();
                // The insertion is the next round's attempt, whatever it returns.
                CHAIN_DEEPEST_ROUND.with(|deepest| deepest.set(deepest.get().max(round + 1)));
                match insert_chain_stations(segments, radius, per_segment, &mut refined, &missing) {
                    Ok(()) => {
                        assign_chain_parameters(&mut refined, segments.len(), per_segment, false);
                        match build_closed_chain_from_samples(
                            solid, segments, radius, chamfer, name, per_segment, fold, bar, refined, round + 1,
                        ) {
                            Ok(Rung::Built(built)) => return Ok(Rung::Built(built)),
                            _ => Some((short, crate::BudgetReason::Rejected)),
                        }
                    }
                    Err(_) => {
                        // The insertion itself was the attempt.
                        CHAIN_DEEPEST_ROUND.with(|deepest| deepest.set(deepest.get().max(round + 1)));
                        Some((short, crate::BudgetReason::Incoherent))
                    }
                }
            };
        }
    }
    // `BREP_DEBUG_CHAIN_DENSE_RAILS=k` (diagnostic only): the accepted rung's
    // rails against exact contacts at k-1 interior fractions of every interval,
    // where `rail_deviation` reads only the halfway one. Nothing reads it to
    // decide.
    if let Some(subdivisions) = std::env::var("BREP_DEBUG_CHAIN_DENSE_RAILS").ok().and_then(|value| value.parse::<usize>().ok()) {
        let contacts = crate::blend::chain::chain_dense_contacts(segments, radius, per_segment, &samples, subdivisions.max(2))?;
        let mut worst = (0.0_f64, 0usize, 0isize, 0.0_f64);
        let mut unreadable = 0usize;
        for (left, next, fraction, p1, p2) in &contacts.solved {
            let (from, to) = (samples[*left].parameter, samples[*next].parameter);
            let global = (from + fraction * (to - from)).rem_euclid(1.0);
            let seed = (global - low) / range;
            let reach = (to - from).abs() / range;
            match rail_pair_miss(closest_on_rail(&cr, *p1, seed, reach).unwrap_or(f64::NAN), closest_on_rail(&cs, *p2, seed, reach).unwrap_or(f64::NAN)) {
                Some(miss) if miss > worst.0 => worst = (miss, samples[*left].segment, samples[*left].position, *fraction),
                Some(_) => {}
                None => unreadable += 1,
            }
        }
        eprintln!(
            "blend chain dense rails: rung {per_segment} round {round}: {} contacts at {subdivisions} per interval, \
             worst {:.9e} (halfway reading {halfway:.9e}, bar {bar:.3e}) at segment {} position {} fraction {:.4}; {} unsolved, {unreadable} unreadable",
            contacts.solved.len(), worst.0, worst.1, worst.2, worst.3, contacts.unsolved
        );
    }
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

    let wall_surface = unmet.as_ref().map(|_| surface.clone());
    let built = chain_surgery(
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
    )?;
    if let (Some((residual, reason)), Some(surface)) = (unmet, wall_surface) {
        crate::blend::record_wall_model_report(crate::blend::WallModelReport {
            name: name.map(str::to_string),
            surface,
            request: model,
            residual,
            detail: request_detail,
            budget: crate::ApproximationBudget {
                reason,
                // Rounds ATTEMPTED (the deepest any refinement of this rung
                // reached); the stations and residual are this shipped rung's.
                rounds_used: CHAIN_DEEPEST_ROUND.with(|deepest| deepest.get()).max(round),
                rounds_limit: model_rounds_limit(),
                stations: stations_now,
                station_limit: CHAIN_CARVE_MAX_PER_SEGMENT,
                mechanism: crate::BudgetMechanism::LocalRounds,
                measured_component: request_component,
                unread: request_unread,
            },
        });
    }
    Ok(Rung::Built(built))
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
pub(in crate::blend) fn project_piece_pcurve(
    piece: &NurbsCurve,
    surface: &NurbsSurface,
    start_is_crossing: bool,
    end_is_crossing: bool,
) -> Result<NurbsCurve, KernelRefusal> {
    project_piece_pcurve_from(piece, surface, start_is_crossing, end_is_crossing, None)
}


/// A trim pcurve `old` as a GUIDE for [`project_piece_pcurve_from`]: at track
/// fraction `f` its uv at the same fraction of its own domain, closed
/// directions wrapped and open ones clamped into `surface`'s domain; `None`
/// where that uv is not finite.
pub(in crate::blend) fn pcurve_guide<'a>(old: &'a NurbsCurve, surface: &NurbsSurface) -> Result<impl Fn(f64) -> Option<[f64; 2]> + 'a, KernelRefusal> {
    let [o0, o1] = old.domain().or_refuse(KernelStage::Refine, "domain")?;
    let (closed_u, closed_v) = surface.closed_directions().or_refuse(KernelStage::Refine, "closed_directions")?;
    let [u0, u1] = surface.domain_u().or_refuse(KernelStage::Refine, "domain_u")?;
    let [v0, v1] = surface.domain_v().or_refuse(KernelStage::Refine, "domain_v")?;
    let into = move |value: f64, closed: bool, low: f64, high: f64| {
        if closed && high > low {
            low + (value - low).rem_euclid(high - low)
        } else {
            value.clamp(low, high)
        }
    };
    Ok(move |fraction: f64| {
        let uv = old.evaluate(o0 + (o1 - o0) * fraction).ok()?;
        let seed = [into(uv.x, closed_u, u0, u1), into(uv.y, closed_v, v0, v1)];
        seed.iter().all(|value| value.is_finite()).then_some(seed)
    })
}

/// [`project_piece_pcurve`], every sample's foot sought from `guide` at its
/// fraction (a uv on the branch the piece is known to run on — a support
/// trim's own march pcurve) instead of from its neighbour's foot. A foot
/// continued from its neighbour stays wherever that neighbour's did: at a
/// STATIONARY edge of the carrier (an extrusion of a profile whose derivative
/// vanishes at its start, S_u = 0 along u = 0) a foot seeded on that edge has
/// no gradient to leave it by, and every later sample stayed there with it
/// (the stationary-start prism's refit: one foot, [0, 0.9], for the whole
/// rail, 6.0 off it at the far end). Without a guide, as before.
pub(in crate::blend) fn project_piece_pcurve_from(
    piece: &NurbsCurve,
    surface: &NurbsSurface,
    start_is_crossing: bool,
    end_is_crossing: bool,
    guide: Option<&dyn Fn(f64) -> Option<[f64; 2]>>,
) -> Result<NurbsCurve, KernelRefusal> {
    project_piece_pcurve_within(piece, surface, start_is_crossing, end_is_crossing, guide, &[])
}

/// [`project_piece_pcurve_from`], its track first refined (two bisections)
/// in the dense interval holding each fraction of `refine_near` — where a
/// caller that read the SHIPPED curve again found it over the floor between
/// the fitter's own stencil points — and then fitted at the same floor.
pub(in crate::blend) fn project_piece_pcurve_within(
    piece: &NurbsCurve,
    surface: &NurbsSurface,
    start_is_crossing: bool,
    end_is_crossing: bool,
    guide: Option<&dyn Fn(f64) -> Option<[f64; 2]>>,
    refine_near: &[f64],
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
    let dense = (8 * piece.control_points.len()).clamp(96, PIECE_TRACK_CEILING);
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
        let guide = guide.filter(|_| {
            true
        });
        // The guide's seed and the neighbour's foot are both tried and the
        // NEARER foot kept: the seeded projector is local and returns a
        // stalled Newton's iterate without saying so, so neither seed is
        // trusted alone (a foot on its carrier is nearer than any stall).
        let seeds: Vec<[f64; 2]> = guide.and_then(|guide| guide(section as f64 / dense as f64)).into_iter().chain(previous_foot).collect();
        let mut projection: Option<crate::projection::SurfaceProjection> = None;
        for seed in &seeds {
            let found = crate::projection::project_point_to_surface_from_seed(surface, point, *seed).or_refuse(KernelStage::Refine, "project_point_to_surface_from_seed")?;
            if projection.as_ref().map_or(true, |known| found.distance < known.distance) {
                projection = Some(found);
            }
        }
        let projection = match projection {
            Some(projection) => projection,
            None => crate::project_point_to_surface(surface, point).or_refuse(KernelStage::Refine, "project_point_to_surface")?,
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
    for &fraction in refine_near {
        for _ in 0..2 {
            let Some(index) = track.fractions.windows(2).position(|pair| pair[0] <= fraction && fraction <= pair[1]) else { break };
            if crate::blend::track_fit::refine_track_intervals(surface, &at, &mut track, &[index], PIECE_TRACK_CEILING)?.is_none() {
                break;
            }
        }
    }
    // Where the top rung still misses the floor, the track is refined ONLY in
    // the dense intervals whose out-of-sample checks missed (local insertion
    // concentrates the rail's stations, and a uniform dense track can leave one
    // sample per knot span where the rail turns hardest), within the dense
    // ceiling the track was built under; every other sample, the pinned ends
    // and the breaks are kept. The floor and the refusal below are unchanged.
    let (fit, inserted) = crate::blend::track_fit::fit_track_locally(
        surface,
        &mut track,
        &at,
        tolerance,
        REFINE_ROUNDS,
        PIECE_TRACK_CEILING,
    )?;
    if std::env::var("BREP_DEBUG_TRACK_FIT").is_ok() {
        eprintln!("TRACK_FIT closed-chain piece: {inserted} sample(s) inserted locally");
        eprintln!(
            "TRACK_FIT closed-chain piece: miss {:.3e} at {} samples of {} ({} breaks), on floor {}",
            fit.miss,
            fit.samples,
            track.fractions.len(),
            track.breaks.len(),
            fit.on_floor
        );
        // Where the miss sits, against what the piece is made of: an endpoint
        // pinned onto a seam reads at fraction 0 or 1; an interior miss names
        // the piece's knot span there and how that span compares with the
        // piece's mean (local insertion clusters spans at the crotch).
        if let Some(worst) = fit.worst {
            let parameter = d0 + (d1 - d0) * worst.fraction;
            let spans: Vec<[f64; 2]> = piece.knots.windows(2)
                .filter(|pair| pair[1] > pair[0] && pair[1] > d0 && pair[0] < d1)
                .map(|pair| [pair[0], pair[1]]).collect();
            let mean = (d1 - d0) / spans.len().max(1) as f64;
            let span = spans.iter().position(|pair| parameter >= pair[0] && parameter <= pair[1]);
            let width = span.map(|index| spans[index][1] - spans[index][0]);
            eprintln!(
                "TRACK_FIT closed-chain worst: fraction {:.9} parameter {:.9} (pinned start {start_is_crossing}, pinned end {end_is_crossing}) \
                 knot span {:?} of {} width {:?} vs mean {:.3e}; dense bracket [{:.9}, {:.9}] (interval {})",
                worst.fraction, parameter, span, spans.len(), width, mean,
                worst.bracket[0], worst.bracket[1], worst.interval
            );
        }
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
