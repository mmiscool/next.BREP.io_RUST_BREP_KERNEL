//! LOCAL RADIUS RELIEF: a constant-radius fillet whose ball does not fit
//! everywhere along its edges keeps the requested radius wherever it fits and
//! reduces it smoothly only where it must, instead of refusing.
//!
//! # The class
//!
//! The stripe network refuses a constant wall that FOLDS (`blend/fold.rs`):
//! the ball's centre curve turns tighter than the ball, or a mate turns
//! tighter than the ball along the contact's track, so the ball pokes out
//! through the face it rides. The three 2026-10-21 reports
//! (`tests/suites/inbox_20261021_fillet_local_relief.rs`):
//!
//! - 37e16630: r 1.7 on a face outline that runs around an r 1 round, so
//!   along that arc no ball over 1 fits;
//! - 5e49bb42: r 2.2 on a cap whose outline turns at r 0.93, so the wall's
//!   rows fold at the turn;
//! - and the same r 1 round at r 1.0 exactly, where the centre curve
//!   collapses to a point and the station Newton is singular.
//!
//! In every case the requested radius fits most of the selection.
//!
//! # The ceiling
//!
//! At stations along each selected edge, the CEILING is the largest ball that
//! solves there with a fold number ([`super::fold::fold_number`]) under
//! [`FOLD_LIMIT`]: continuation upward from a small ball to the requested
//! one, then bisection between the last ball that fitted and the first that
//! did not. Where the requested ball fits, the ceiling is the requested radius.
//!
//! # The law
//!
//! The target is `min(R, MARGIN · ceiling)` at every sample of every selected
//! edge, read on one graph of the selection (samples along each edge, the
//! edges joined at their shared vertices), so a relief reaches across a vertex
//! into the next edge and every edge meeting at a vertex has one radius there
//! (the network solves its corners at the laws' END radii).
//!
//! - A RELIEF ZONE is a connected run of samples whose target is under R.
//!   Inside it the radius is the zone's least target: a flat plateau.
//! - Outside it the radius climbs back to R over the ramp length
//!   `λ = (R − m) / RAMP_SLOPE` measured along the graph, as the quintic
//!   smoothstep `m + (R − m) S(d / λ)`. `S` is C2 at both ends, so the law is
//!   C1 (C2) across both joins, and its steepest slope is `1.875 (R − m) / λ`.
//! - Two zones whose ramps would meet are merged into one, with the shortest
//!   path between them and the lesser plateau. Two ramps crossing below R
//!   would leave a kink, and the merged plateau has none.
//!
//! The law is sampled into each edge's stop list and built by the taper
//! network (`fillet_edges_variable_impl`), whose own fold check judges the
//! relieved wall. A relieved wall that still folds is planned again with a
//! smaller margin and longer ramps, a bounded number of times
//! ([`ATTEMPTS`]).
//!
//! `BREP_BLEND_RADIUS_RELIEF=0`/`off` is the escape hatch: the fold refuses
//! as it did before 2026-10-21, and a support cusped under the edge's end is
//! neither admitted by the support check nor tapered before the free end
//! (`blend/taper_runout.rs`, report b46db89a).

use crate::topology::{BrepSolid, EdgeRecord};
use crate::{KernelRefusal, KernelStage, OrRefuse, Vec3};

use super::stations::{edge_uv_on_face, locate_mate, march_model_scale, march_section, signed_radii, solve_station, tangency_residual};

/// A ball fits a station when its fold number is under this. One is the
/// fold itself. The margin below it keeps the relieved wall's sections a
/// visible distance from the fold, where the fitted rows still read clean.
pub(crate) const FOLD_LIMIT: f64 = 0.9;

/// The plateau sits at this share of the ceiling (which already stands
/// [`FOLD_LIMIT`] short of the fold); later attempts keep more distance.
const MARGIN: [f64; ATTEMPTS] = [1.0, 0.8, 0.8, 0.65];

/// The ramp's mean slope (radius per unit length along the edges); the
/// smoothstep's steepest is 1.875 times this. Steep first, to keep the
/// requested radius on as much of the edge as builds: report 37e16630 builds
/// at 0.25, while report 5e49bb42's sculpted wall misses the pcurve floor at
/// 0.25 and 0.15 (1.7e-5, 4.8e-6) and builds at 0.1.
const RAMP_SLOPE: [f64; ATTEMPTS] = [0.25, 0.15, 0.1, 0.1];

/// How many plans a relief tries before it gives up.
pub(crate) const ATTEMPTS: usize = 4;

/// Ceiling stations per edge, at the midpoints of equal parameter intervals.
const CEILING_STATIONS: usize = 48;

/// Continuation steps from a small ball up to the requested one, then the
/// bisection steps between the last that fitted and the first that did not.
const CONTINUATION_STEPS: usize = 12;
const BISECTIONS: usize = 7;

/// Every point of the ball's section must advance with the ball by at least
/// this share of the edge's own advance: a contact rail that slows is turning
/// tighter than the outline it follows (around a round of radius ρ a rail at
/// offset `d` advances `1 − d/ρ`) and stops at its cusp, before any fold
/// verdict reads it (report 5e49bb42's cap rail, at its tightest station:
/// 0.40 at r 0.92, 0.27 at r 1.10, 0.15 at r 1.28, 0.004 at r 1.47). The
/// FITTED rows loop sooner where the turn is short (r 1.2 there): that is the
/// network's rows-loop refusal, which the next attempt's [`MARGIN`] answers.
const SECTION_ADVANCE_FLOOR: f64 = 0.1;

/// Points read across each section by the advance test, contacts included.
const SECTION_SAMPLES: usize = 8;

/// Sub-stations per station interval read around a short station.
const REFINE: usize = 4;

/// A station whose ball reads at least this share of the way to the margins
/// (its fold number against `fold_limit`, its section slowdown against
/// `1 − advance_floor`, the largest over the sweep), and no less than either
/// neighbour, is refined like a short station: a turn narrower than the
/// station spacing can fold between two stations that both read clear.
/// Report 93aa19e7 (2026-10-25, r 1.2 on the sculpted cap's rim, measured
/// 2026-10-26): the apex of the cap outline's r 0.93 turn lies at 0.46615 of
/// the rim, where the r 1.2 ball reads fold number 13.96 and section advance
/// −2.33; the stations either side of it read 0.86 / 0.19 (0.46484) and
/// 0.85 / 0.19 (0.46745), the base stations 0.82 / 0.21 (0.46875) and
/// 0.43 / 0.56 (0.48958), so neither the 48 stations nor one level of
/// refinement read anything short, the relief never ran, and the march's
/// own refusal stood (the closed march's Newton on the joined rim, the rows'
/// fold verdict on the two-edge rim).
const REFINE_STRESS: f64 = 0.5;

/// Further refinement levels around the stress peak while nothing reads
/// short (see [`station_ceilings`]): the apex of a turn narrower than the
/// refined spacing.
const PEAK_LEVELS: usize = 2;

/// Law samples per edge (stops), endpoints included. The taper network
/// interpolates them C1 (PCHIP), so its rails are only as smooth as the
/// samples are dense: on report 5e49bb42 the relieved rim's pcurve missed its
/// 1e-7 floor by 2.2e-5 at 64 samples and 4.4e-7 at 256.
const LAW_SAMPLES: usize = 1024;

const RELIEF_DEFAULT: bool = true;

thread_local! {
    static RELIEF_FOR_TEST: std::cell::Cell<Option<bool>> = const { std::cell::Cell::new(None) };
    /// Set while a relief builds, so the constant builds it makes do not
    /// relieve again.
    static RELIEVING: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
    static MISFIT_OFF: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
}

/// `BREP_BLEND_RADIUS_RELIEF=1`/`on` (the default) relieves a folding
/// constant-radius fillet; `=0`/`off` refuses it as before.
pub(crate) fn radius_relief_on() -> bool {
    !RELIEVING.with(|active| active.get()) && relief_switch()
}

/// The taper network's flush join at a varying radius (a stripe whose law
/// ramps through a smooth vertex, solved at its END radius there), which the
/// relief's ramps need: on with the relief, and off under its escape hatch,
/// where it refuses as before.
pub(crate) fn tapered_flush_join_on() -> bool {
    relief_switch()
}

fn relief_switch() -> bool {
    if let Some(forced) = RELIEF_FOR_TEST.with(|forced| forced.get()) {
        return forced;
    }
    match std::env::var("BREP_BLEND_RADIUS_RELIEF").as_deref() {
        Ok("0") | Ok("off") => false,
        Ok("1") | Ok("on") => true,
        _ => RELIEF_DEFAULT,
    }
}


/// The relief's switch, whether or not a relief is building now.
pub(crate) fn relief_enabled() -> bool {
    relief_switch()
}

/// Is a relief building on this thread ([`Relieving`])?
pub(crate) fn relieving() -> bool {
    RELIEVING.with(|active| active.get())
}

/// Held while a relief builds: [`radius_relief_on`] reads false under it.
pub(crate) struct Relieving;

impl Relieving {
    pub(crate) fn enter() -> Relieving {
        RELIEVING.with(|active| active.set(true));
        Relieving
    }
}

impl Drop for Relieving {
    fn drop(&mut self) {
        RELIEVING.with(|active| active.set(false));
    }
}

/// `BREP_DEBUG_RELIEF=1`: every ceiling probe's reading.
fn trace_on() -> bool {
    std::env::var("BREP_DEBUG_RELIEF").is_ok()
}

fn debug_on() -> bool {
    std::env::var("BREP_DEBUG_NETWORK").is_ok()
}

/// One edge's ceilings: `(edge fraction, largest ball that fits)` at the
/// [`CEILING_STATIONS`] station midpoints. A station where even the smallest
/// ball does not solve reads zero.
pub(crate) fn edge_ceilings(solid: &BrepSolid, edge: &EdgeRecord, radius: f64) -> Result<Vec<(f64, f64)>, KernelRefusal> {
    Ok(station_ceilings(solid, edge, radius, false)?.into_iter().map(|(fraction, ceiling, _)| (fraction, ceiling)).collect())
}

/// [`edge_ceilings`] with each station's reason: `true` when the first ball
/// past its ceiling SOLVED and folds (its fold number or section advance),
/// `false` when it did not solve or could not be read — a station Newton's
/// limit, not the shape's (the 20-cube's opposite pair at r 20 reads 9.99,
/// a full-width bar at 3.5e-7 short reads 0).
///
/// `strict` reads the fold itself (fold number 1, a section point standing
/// still) instead of the relief's margins ([`FOLD_LIMIT`],
/// [`SECTION_ADVANCE_FLOOR`]): whether the requested ball folds at all, as
/// opposed to how far below it the relief keeps the wall.
fn station_ceilings(solid: &BrepSolid, edge: &EdgeRecord, radius: f64, strict: bool) -> Result<Vec<(f64, f64, bool)>, KernelRefusal> {
    let (fold_limit, advance_floor) = if strict { (1.0, 0.0) } else { (FOLD_LIMIT, SECTION_ADVANCE_FLOOR) };
    let (first_face, first_loop, first_coedge) = locate_mate(solid, edge.id, None)?;
    let (second_face, _, second_coedge) = locate_mate(solid, edge.id, Some((first_face.id, first_loop)))?;
    let (rho1, rho2) = signed_radii(edge, first_face, first_coedge, second_face, second_coedge, radius)?;
    let signs = [rho1.signum(), rho2.signum()];
    let surfaces = [&first_face.surface, &second_face.surface];
    let span = edge.t1 - edge.t0;
    let scale = march_model_scale(&edge.curve, edge.t0, edge.t1, radius)?;
    let step = span / CEILING_STATIONS as f64;
    let ceiling_at = |fraction: f64| -> Result<(f64, bool, f64), KernelRefusal> {
        let t = edge.t0 + span * fraction;
        let at_edge = [edge_uv_on_face(first_coedge, edge, t)?, edge_uv_on_face(second_coedge, edge, t)?];
        let seed0 = [at_edge[0][0], at_edge[0][1], at_edge[1][0], at_edge[1][1]];
        // The station at `r`, seeded from `seed`: its unknowns, or `None`.
        let solve_at = |r: f64, seed: [f64; 4]| -> Option<[f64; 4]> {
            let (point, tangent) = march_section(edge, t, step, "radius relief").ok()?;
            solve_station(surfaces[0], surfaces[1], [signs[0] * r, signs[1] * r], seed, point, tangent, scale).ok()
        };
        // The largest stress of a ball that fit here (see [`REFINE_STRESS`]).
        let stress = std::cell::Cell::new(0.0_f64);
        // Does a ball of `r` fit, seeded from `seed` (the last ball that did)?
        // Its unknowns if so; if not, whether it solved and folds.
        let fits = |r: f64, seed: [f64; 4]| -> Result<[f64; 4], bool> {
            let uv = solve_at(r, seed);
            if uv.is_none() && trace_on() {
                eprintln!("relief probe: edge {} fraction {fraction:.5} r {r:.5} does not solve", edge.id);
            }
            let uv = uv.ok_or(false)?;
            let read = || -> Option<bool> {
                let rho = [signs[0] * r, signs[1] * r];
                let probe = |at: f64, seed: [f64; 4]| -> Result<([f64; 4], Vec3, Vec3, Vec3), KernelRefusal> {
                    let (point, tangent) = march_section(edge, at, step, "radius relief")?;
                    let uv = solve_station(surfaces[0], surfaces[1], rho, seed, point, tangent, scale)?;
                    let (_, p1, p2, centre) = tangency_residual(surfaces[0], surfaces[1], rho, uv, point, tangent)?;
                    Ok((uv, p1, p2, centre))
                };
                let reading = super::fold::fold_number(r, span, t, uv, &probe, surfaces, scale);
                if trace_on() {
                    eprintln!("relief probe: edge {} fraction {fraction:.5} r {r:.5} reading {reading:?}", edge.id);
                }
                let reading = reading?;
                // Every section point must advance with the ball, by
                // SECTION_ADVANCE_FLOOR of the edge's own advance at least
                // (`check_rows_loop`'s test, on the exact sections).
                let step = span * super::fold::PROBE_FRACTION;
                let (before, after) = (march_section(edge, t - step, step, "radius relief").ok()?, march_section(edge, t + step, step, "radius relief").ok()?);
                let along = after.0.sub(before.0).length();
                let [(before_contacts, before_centre), (after_contacts, after_centre)] = reading.sections;
                let ball = after_centre.sub(before_centre).normalized().ok()?;
                let advance = section_points(before_contacts, before_centre)?
                    .iter()
                    .zip(&section_points(after_contacts, after_centre)?)
                    .map(|(a, b)| b.sub(*a).dot(ball) / along)
                    .fold(f64::INFINITY, f64::min);
                if trace_on() {
                    eprintln!("relief probe: edge {} fraction {fraction:.5} r {r:.5} section advance {advance:.6}", edge.id);
                }
                let fit = reading.number < fold_limit && advance > advance_floor;
                if fit {
                    // The largest over the sweep, not the last ball's: at
                    // r 1.7 the requested ball spans the sculpted cap's
                    // 0.93 turn and reads 0.55 at its apex, where the r 1.4
                    // ball read 0.95 (report 67598fe2).
                    stress.set(stress.get().max((reading.number / fold_limit).max((1.0 - advance) / (1.0 - advance_floor))));
                }
                Some(fit)
            };
            match read() {
                Some(true) => Ok(uv),
                Some(false) => Err(true),
                None => Err(false),
            }
        };
        let mut seed = seed0;
        let mut good = 0.0;
        let mut bad = None;
        for step_index in 1..=CONTINUATION_STEPS {
            let r = radius * step_index as f64 / CONTINUATION_STEPS as f64;
            match fits(r, seed) {
                Ok(uv) => {
                    good = r;
                    seed = uv;
                }
                Err(folds) => {
                    bad = Some((r, folds));
                    break;
                }
            }
        }
        // A limit that did not SOLVE says nothing about the shape on its
        // own; a ball a step or more beyond it that solves and folds does
        // (the sculpted cap's apex at r 1.4: 1.167 does not solve, 1.283
        // folds 7.6), and the station's reason is then that fold.
        if let Some((first_bad, false)) = bad {
            let mut step_index = (first_bad / radius * CONTINUATION_STEPS as f64).round() as usize + 1;
            while step_index <= CONTINUATION_STEPS {
                let r = radius * step_index as f64 / CONTINUATION_STEPS as f64;
                if let Err(true) = fits(r, seed) {
                    bad = Some((first_bad, true));
                    break;
                }
                step_index += 1;
            }
        }
        let ceiling = match bad {
            None => (radius, false),
            Some((mut high, mut folds)) => {
                for _ in 0..BISECTIONS {
                    let middle = 0.5 * (good + high);
                    match fits(middle, seed) {
                        Ok(uv) => {
                            good = middle;
                            seed = uv;
                        }
                        Err(why) => {
                            high = middle;
                            // A fold read anywhere past the ceiling stays
                            // the station's reason: the probes nearest the
                            // ceiling sit on the fold's own singular threshold
                            // and may not solve (the sculpted cap's apex at
                            // r 2.2: 1.283 solves and folds 7.6, 1.192 folds
                            // 15.5, 1.146 and 1.143 do not solve, 1.142 fits),
                            // which says nothing against the fold read beyond.
                            folds |= why;
                        }
                    }
                }
                (good, folds)
            }
        };
        Ok((ceiling.0, ceiling.1, stress.get()))
    };
    // Every station read: (fraction, ceiling, folds, stress).
    let mut all: Vec<(f64, f64, bool, f64)> = Vec::with_capacity(CEILING_STATIONS + 4 * REFINE * (PEAK_LEVELS + 1));
    for index in 0..CEILING_STATIONS {
        let fraction = (index as f64 + 0.5) / CEILING_STATIONS as f64;
        let (ceiling, folds, stress) = ceiling_at(fraction)?;
        all.push((fraction, ceiling, folds, stress));
    }
    // A turn tighter than the station spacing reads short at one station or
    // none: around every short station, and every station whose stress peaks
    // near the margins ([`REFINE_STRESS`]), the stations either side are read
    // again at a [`REFINE`]-fold spacing, and the law takes the least.
    let peak = |index: usize| -> bool {
        let around = |other: Option<usize>| other.and_then(|other| all.get(other)).map(|s| s.3).unwrap_or(0.0);
        all[index].3 >= REFINE_STRESS && all[index].3 >= around(index.checked_sub(1)) && all[index].3 >= around(Some(index + 1))
    };
    let short: Vec<usize> = (0..CEILING_STATIONS).filter(|index| all[*index].1 < radius || peak(*index)).collect();
    let mut refined = Vec::new();
    for index in short {
        let low = (index as f64 - 1.0 + 0.5).max(0.0) / CEILING_STATIONS as f64;
        let high = (index as f64 + 1.0 + 0.5).min(CEILING_STATIONS as f64) / CEILING_STATIONS as f64;
        for sub in 1..2 * REFINE {
            let fraction = low + (high - low) * sub as f64 / (2 * REFINE) as f64;
            if all.iter().chain(&refined).any(|s: &(f64, f64, bool, f64)| (s.0 - fraction).abs() < 1e-12) {
                continue;
            }
            let (ceiling, folds, stress) = ceiling_at(fraction)?;
            refined.push((fraction, ceiling, folds, stress));
        }
    }
    all.extend(refined);
    // A turn narrower than that spacing still reads clear either side of
    // its apex (report 93aa19e7 at r 1.2: fold number 0.82 and advance 0.21
    // at 0.46875 and 0.47396 of the rim, 13.96 and −2.33 at 0.46615 between
    // them): while no station reads short and the stress still peaks, two
    // spacings either side of the peak are read at a [`REFINE`]-fold finer
    // spacing again, [`PEAK_LEVELS`] times.
    let mut spacing = 1.0 / (CEILING_STATIONS as f64 * REFINE as f64);
    for _ in 0..PEAK_LEVELS {
        if all.iter().any(|s| s.1 < radius) {
            break;
        }
        let Some(&(peak_fraction, _, _, _)) = all.iter().filter(|s| s.3 >= REFINE_STRESS).max_by(|a, b| a.3.total_cmp(&b.3)) else {
            break;
        };
        spacing /= REFINE as f64;
        for k in 1..=2 * REFINE as i64 {
            for fraction in [peak_fraction - k as f64 * spacing, peak_fraction + k as f64 * spacing] {
                if !(0.0..=1.0).contains(&fraction) || all.iter().any(|s| (s.0 - fraction).abs() < 1e-12) {
                    continue;
                }
                let (ceiling, folds, stress) = ceiling_at(fraction)?;
                all.push((fraction, ceiling, folds, stress));
            }
        }
    }
    let mut ceilings: Vec<(f64, f64, bool)> = all.into_iter().map(|(f, c, folds, _)| (f, c, folds)).collect();
    ceilings.sort_by(|a, b| a.0.total_cmp(&b.0));
    if debug_on() {
        let least = ceilings.iter().map(|(_, c, _)| *c).fold(f64::INFINITY, f64::min);
        let short: Vec<String> = ceilings
            .iter()
            .filter(|(_, c, _)| *c < radius)
            .map(|(f, c, folds)| format!("{f:.3}:{c:.3}{}", if *folds { "" } else { "?" }))
            .collect();
        eprintln!("radius relief: edge {} ceilings at r {radius}: least {least:.6} short {}", edge.id, short.join(" "));
    }
    Ok(ceilings)
}

/// [`SECTION_SAMPLES`] points across the ball's section arc, from one contact
/// to the other around `centre`.
fn section_points(contacts: [Vec3; 2], centre: Vec3) -> Option<Vec<Vec3>> {
    let (a, b) = (contacts[0].sub(centre), contacts[1].sub(centre));
    let radius = a.length();
    let (unit_a, unit_b) = (a.normalized().ok()?, b.normalized().ok()?);
    let angle = unit_a.cross(unit_b).length().atan2(unit_a.dot(unit_b));
    let normal = unit_a.cross(unit_b).normalized().ok()?;
    let across = normal.cross(unit_a);
    Some(
        (0..SECTION_SAMPLES)
            .map(|k| {
                let phi = angle * k as f64 / (SECTION_SAMPLES - 1) as f64;
                centre.add(unit_a.scale(radius * phi.cos())).add(across.scale(radius * phi.sin()))
            })
            .collect(),
    )
}

/// Every selected edge's ceilings ([`edge_ceilings`]), in selection order:
/// measured once and read by every attempt's [`plan_relief`].
pub(crate) fn measure_ceilings(solid: &BrepSolid, edge_ids: &[u64], radius: f64) -> Result<Vec<Vec<(f64, f64)>>, KernelRefusal> {
    edge_ids
        .iter()
        .map(|id| {
            let edge = solid.edges.iter().find(|edge| edge.id == *id).ok_or_else(|| {
                KernelRefusal::internal(KernelStage::Collect, "relief_edge", format!("radius relief: edge {id} not found"))
            })?;
            edge_ceilings(solid, edge, radius)
        })
        .collect()
}

/// The selection with its CONTINUATION edges joined: where exactly two
/// selected edges meet at a vertex as one carrier continued (the same two
/// faces, tangent — `coalesce::merge_curve_continuation_edges_at`'s own
/// test), they become one edge. A face's outline split into two edges of one
/// curve (report 5e49bb42's cap rim: 0.27 and 22.85 long) is then one closed
/// edge, whose law the closed march carries round (`blend_edge_variable`)
/// where the network cannot march a tapered stripe across that vertex. `None`
/// when nothing joins.
pub(crate) fn join_continuations(solid: &BrepSolid, edge_ids: &[u64]) -> Option<BrepSolid> {
    let mut uses: Vec<(u64, usize)> = Vec::new();
    for id in edge_ids {
        let edge = solid.edges.iter().find(|edge| edge.id == *id)?;
        if edge.start_vertex_id == edge.end_vertex_id {
            continue;
        }
        for vertex in [edge.start_vertex_id, edge.end_vertex_id] {
            match uses.iter_mut().find(|(v, _)| *v == vertex) {
                Some((_, count)) => *count += 1,
                None => uses.push((vertex, 1)),
            }
        }
    }
    let at: rustc_hash::FxHashSet<u64> = uses.iter().filter(|(_, count)| *count == 2).map(|(vertex, _)| *vertex).collect();
    if at.is_empty() {
        return None;
    }
    let tolerance = crate::KernelTolerances::for_solid(solid, crate::BooleanOptions::default().tolerance).model;
    let joined = crate::coalesce::merge_curve_continuation_edges_at(solid, tolerance, &at).ok()?;
    (joined.edges.len() < solid.edges.len() && joined.validate().is_empty()).then_some(joined)
}

/// Does the requested ball FOLD somewhere on these edges, while it fits
/// elsewhere on them? The first edge where it folds, with its least strict
/// ceiling among the stations whose limit is a fold (`station_ceilings`): a
/// station whose ball merely does not solve says nothing about the shape, and
/// the network's own refusal stands for it. A selection the ball fits NOWHERE
/// is not a local relief: a full round of a rod end (the ball the size of the
/// rim) has its own lanes past the network, which build it to the closed
/// form.
pub(crate) fn first_misfit(solid: &BrepSolid, edge_ids: &[u64], radius: f64) -> Option<(u64, f64)> {
    if MISFIT_OFF.with(|off| off.get()) {
        return None;
    }
    let mut misfit = None;
    let mut fits_somewhere = false;
    for &edge_id in edge_ids {
        let edge = solid.edges.iter().find(|edge| edge.id == edge_id)?;
        let Ok(ceilings) = station_ceilings(solid, edge, radius, true) else {
            continue;
        };
        fits_somewhere |= ceilings.iter().any(|(_, c, _)| *c >= radius);
        let least = ceilings
            .iter()
            .filter(|(_, c, folds)| *folds && *c > 0.0)
            .map(|(_, c, _)| *c)
            .fold(f64::INFINITY, f64::min);
        if least < radius && misfit.is_none() {
            misfit = Some((edge_id, least));
        }
    }
    misfit.filter(|_| fits_somewhere)
}

/// Is this the fold [`misfit_fold`] minted?
pub(crate) fn is_misfit_fold(refusal: &KernelRefusal) -> bool {
    refusal.message.contains(MISFIT_MARK)
}

const MISFIT_MARK: &str = "the largest ball that rolls without folding is";

/// Held while a fillet whose misfit relief built nothing runs the path the
/// misfit pre-empted: [`first_misfit`] reads `None` under it.
pub(crate) struct MisfitOff;

impl MisfitOff {
    pub(crate) fn enter() -> MisfitOff {
        MISFIT_OFF.with(|off| off.set(true));
        MisfitOff
    }
}

impl Drop for MisfitOff {
    fn drop(&mut self) {
        MISFIT_OFF.with(|off| off.set(false));
    }
}

/// One relief zone of a plan: where the radius is reduced, and to what.
#[derive(Clone, Debug)]
pub(crate) struct ReliefZone {
    /// The plateau radius.
    pub(crate) radius: f64,
    /// The selected edges (indices into the selection) the plateau lies on.
    pub(crate) edges: Vec<usize>,
    /// The least ceiling measured in the zone, before the margin.
    pub(crate) ceiling: f64,
}

/// A planned relief: one stop list per selected edge (`(edge fraction,
/// radius)`, endpoints included) and the zones it reduces.
#[derive(Clone, Debug)]
pub(crate) struct ReliefPlan {
    pub(crate) stops: Vec<Vec<(f64, f64)>>,
    pub(crate) zones: Vec<ReliefZone>,
}

/// The law's graph: every edge's samples, the end samples of edges that
/// share a vertex being one node.
struct LawGraph {
    /// Per edge, its sample nodes in fraction order (`LAW_SAMPLES + 1`).
    nodes: Vec<Vec<usize>>,
    /// Per node, its neighbours and the arc length to each.
    adjacency: Vec<Vec<(usize, f64)>>,
}

fn law_graph(edges: &[&EdgeRecord]) -> Result<LawGraph, KernelRefusal> {
    let mut vertex_nodes: Vec<(u64, usize)> = Vec::new();
    let mut count = 0usize;
    let mut node_of_vertex = |vertex: u64, count: &mut usize| -> usize {
        if let Some((_, node)) = vertex_nodes.iter().find(|(id, _)| *id == vertex) {
            return *node;
        }
        let node = *count;
        *count += 1;
        vertex_nodes.push((vertex, node));
        node
    };
    let mut nodes = Vec::with_capacity(edges.len());
    let mut lengths = Vec::with_capacity(edges.len());
    for edge in edges {
        let mut row = Vec::with_capacity(LAW_SAMPLES + 1);
        let closed = edge.start_vertex_id == edge.end_vertex_id;
        row.push(node_of_vertex(edge.start_vertex_id, &mut count));
        for _ in 1..LAW_SAMPLES {
            row.push(count);
            count += 1;
        }
        row.push(if closed { row[0] } else { node_of_vertex(edge.end_vertex_id, &mut count) });
        let mut spans = Vec::with_capacity(LAW_SAMPLES);
        let mut previous = edge.curve.evaluate(edge.t0).or_refuse(KernelStage::Refine, "evaluate")?;
        for index in 1..=LAW_SAMPLES {
            let t = edge.t0 + (edge.t1 - edge.t0) * index as f64 / LAW_SAMPLES as f64;
            let point = edge.curve.evaluate(t).or_refuse(KernelStage::Refine, "evaluate")?;
            spans.push(point.sub(previous).length());
            previous = point;
        }
        nodes.push(row);
        lengths.push(spans);
    }
    let mut adjacency = vec![Vec::new(); count];
    for (row, spans) in nodes.iter().zip(&lengths) {
        for index in 0..LAW_SAMPLES {
            let (a, b) = (row[index], row[index + 1]);
            adjacency[a].push((b, spans[index]));
            adjacency[b].push((a, spans[index]));
        }
    }
    Ok(LawGraph { nodes, adjacency })
}

/// Graph distance from every node to the nearest of `sources`.
fn distances(graph: &LawGraph, sources: &[usize]) -> Vec<f64> {
    let mut distance = vec![f64::INFINITY; graph.adjacency.len()];
    let mut heap = std::collections::BinaryHeap::new();
    for &source in sources {
        distance[source] = 0.0;
        heap.push(std::cmp::Reverse((0u64, source)));
    }
    while let Some(std::cmp::Reverse((bits, node))) = heap.pop() {
        let here = f64::from_bits(bits);
        if here > distance[node] {
            continue;
        }
        for &(next, length) in &graph.adjacency[node] {
            let through = here + length;
            if through < distance[next] {
                distance[next] = through;
                heap.push(std::cmp::Reverse((through.to_bits(), next)));
            }
        }
    }
    distance
}

/// The quintic smoothstep: 0 at 0, 1 at 1, first and second derivatives zero
/// at both.
fn smoothstep(x: f64) -> f64 {
    let x = x.clamp(0.0, 1.0);
    x * x * x * (x * (6.0 * x - 15.0) + 10.0)
}

/// Plan the relief of a constant `radius` over the selected `edge_ids`
/// (attempt `attempt` of [`ATTEMPTS`]: later ones keep a wider margin and
/// ramp more gently). `None` when the ball fits everywhere, so there is
/// nothing to relieve.
pub(crate) fn plan_relief(
    solid: &BrepSolid,
    edge_ids: &[u64],
    radius: f64,
    measured: &[Vec<(f64, f64)>],
    attempt: usize,
) -> Result<Option<ReliefPlan>, KernelRefusal> {
    let attempt = attempt.min(ATTEMPTS - 1);
    let edges: Vec<&EdgeRecord> = edge_ids
        .iter()
        .map(|id| {
            solid.edges.iter().find(|edge| edge.id == *id).ok_or_else(|| {
                KernelRefusal::internal(KernelStage::Collect, "relief_edge", format!("radius relief: edge {id} not found"))
            })
        })
        .collect::<Result<_, _>>()?;
    let graph = law_graph(&edges)?;
    let node_count = graph.adjacency.len();
    // The target at every node: min(R, margin · ceiling), the ceiling read
    // off the nearest station (a vertex node takes the least of its edges').
    let mut ceiling = vec![f64::INFINITY; node_count];
    for (index, stations) in measured.iter().enumerate() {
        for (sample, &node) in graph.nodes[index].iter().enumerate() {
            let fraction = sample as f64 / LAW_SAMPLES as f64;
            // The least reading within one law sample either side: the law
            // never trusts the larger of two neighbouring readings across an
            // unread stretch, and a refined dip narrower than a law sample
            // still reaches the samples beside it.
            let reach = 1.0 / LAW_SAMPLES as f64 + 0.5 / CEILING_STATIONS as f64;
            let value = stations
                .iter()
                .filter(|(f, _)| (f - fraction).abs() <= reach)
                .map(|(_, c)| *c)
                .fold(f64::INFINITY, f64::min);
            let value = if value.is_finite() {
                value
            } else {
                stations.iter().min_by(|a, b| (a.0 - fraction).abs().total_cmp(&(b.0 - fraction).abs())).map(|(_, c)| *c).unwrap_or(radius)
            };
            ceiling[node] = ceiling[node].min(value);
        }
    }
    let target: Vec<f64> = ceiling.iter().map(|c| (MARGIN[attempt] * c).min(radius)).collect();
    let short = |node: usize| ceiling[node] < radius;
    // Zones: connected components of short nodes.
    let mut zone_of = vec![usize::MAX; node_count];
    let mut zones: Vec<Vec<usize>> = Vec::new();
    for start in 0..node_count {
        if !short(start) || zone_of[start] != usize::MAX {
            continue;
        }
        let id = zones.len();
        let mut members = vec![start];
        zone_of[start] = id;
        let mut cursor = 0;
        while cursor < members.len() {
            let node = members[cursor];
            cursor += 1;
            for &(next, _) in &graph.adjacency[node] {
                if short(next) && zone_of[next] == usize::MAX {
                    zone_of[next] = id;
                    members.push(next);
                }
            }
        }
        zones.push(members);
    }
    if zones.is_empty() {
        return Ok(None);
    }
    let plateau = |members: &[usize]| members.iter().map(|&n| target[n]).fold(f64::INFINITY, f64::min);
    let ramp = |level: f64| (radius - level) / RAMP_SLOPE[attempt];
    // Merge zones whose ramps would meet.
    loop {
        let fields: Vec<Vec<f64>> = zones.iter().map(|members| distances(&graph, members)).collect();
        let mut merged = None;
        'pairs: for a in 0..zones.len() {
            for b in a + 1..zones.len() {
                let gap = zones[b].iter().map(|&n| fields[a][n]).fold(f64::INFINITY, f64::min);
                if gap < ramp(plateau(&zones[a])) + ramp(plateau(&zones[b])) {
                    merged = Some((a, b, gap));
                    break 'pairs;
                }
            }
        }
        let Some((a, b, gap)) = merged else {
            break;
        };
        let mut members: Vec<usize> = (0..node_count)
            .filter(|&n| fields[a][n] + fields[b][n] <= gap * (1.0 + 1e-9) + 1e-12)
            .collect();
        members.extend(zones[a].iter().chain(&zones[b]).copied());
        members.sort_unstable();
        members.dedup();
        zones.remove(b);
        zones[a] = members;
    }
    let levels: Vec<f64> = zones.iter().map(|members| plateau(members)).collect();
    if let Some(level) = levels.iter().find(|level| !(**level > 0.0)) {
        return Err(KernelRefusal::unsupported(
            KernelStage::Classify,
            "relief_nothing_fits",
            format!("radius relief: no ball fits somewhere along the selection (plateau {level})"),
        ));
    }
    let fields: Vec<Vec<f64>> = zones.iter().map(|members| distances(&graph, members)).collect();
    let law = |node: usize| -> f64 {
        let mut value = radius;
        for (zone, level) in levels.iter().enumerate() {
            let length = ramp(*level);
            let reading = level + (radius - level) * smoothstep(fields[zone][node] / length);
            value = value.min(reading);
        }
        value
    };
    let stops: Vec<Vec<(f64, f64)>> = graph
        .nodes
        .iter()
        .map(|row| {
            row.iter()
                .enumerate()
                .map(|(sample, &node)| (sample as f64 / LAW_SAMPLES as f64, law(node)))
                .collect()
        })
        .collect();
    let zones = zones
        .iter()
        .zip(&levels)
        .map(|(members, level)| ReliefZone {
            radius: *level,
            edges: graph
                .nodes
                .iter()
                .enumerate()
                .filter(|(_, row)| row.iter().any(|node| members.contains(node)))
                .map(|(index, _)| index)
                .collect(),
            ceiling: members.iter().map(|&n| ceiling[n]).fold(f64::INFINITY, f64::min),
        })
        .collect();
    if debug_on() {
        for (index, row) in stops.iter().enumerate() {
            let least = row.iter().map(|(_, r)| *r).fold(f64::INFINITY, f64::min);
            let reduced: Vec<String> = row.iter().filter(|(_, r)| *r < radius).map(|(f, r)| format!("{f:.3}:{r:.3}")).collect();
            eprintln!("radius relief: edge {} law least {least:.6} (attempt {attempt}) reduced {}", edge_ids[index], reduced.join(" "));
        }
    }
    Ok(Some(ReliefPlan { stops, zones }))
}

/// The fold a network refusal that was not one stands for, when the ball
/// does not fit (`first_misfit`): minted as a wall fold, so the group stops
/// on it and the fillet's entry relieves it.
pub(crate) fn misfit_fold(edge_id: u64, least: f64, radius: f64, refusal: &KernelRefusal) -> KernelRefusal {
    super::fold::wall_fold_refusal(
        Vec::new(),
        format!(
            "{} {radius} fits this edge: along edge {edge_id} {MISFIT_MARK} {least:.6} (the network refused: {refusal})",
            super::fold::WALL_FOLDS
        ),
    )
}
