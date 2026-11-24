//! `BREP_STEP_PREFIT` (default on; `0` off, `check` compares): the projection
//! fits of a large body's coedges, run on idle threads ahead of the face loop.
//!
//! A body's faces are finished one after another on the thread that builds the
//! body, and on a large part (a 2171-face duct) the coedges' projection fits
//! are most of that time while every other thread has run out of bodies. Each
//! fit is a function of its inputs alone — the carrier, the edge's curve and
//! range, the sense and the tolerance — so it can be made anywhere and give
//! the same curve and report.
//!
//! Before the face loop, one job per face is SPAWNED (never waited on). A job
//! fits its face's coedges as the loop would: the size-derived ask, then the
//! floor refit exactly when the loop's trigger would ask for it. The loop
//! claims each face's slot when it reaches it: a finished job's fits are
//! offered to that face's `build_loop`; an unstarted or running job is
//! claimed and the face is fitted on the loop's own thread, as before. So the
//! loop never waits, and a body whose helpers never ran is built exactly as
//! today. A fit is used only when every input matches bit for bit (the
//! carrier's net and knots, the edge's curve and range, sense, tolerance);
//! an edge that the face's own seam stitch changed misses and is fitted fresh.
//!
//! Fits made on a helper are not tallied by a fit ledger open on the loop's
//! thread; no STEP import runs under one. `check` fits every coedge on the
//! loop's thread as before and compares it with the helper's, printing the
//! comparison under `BREP_PROFILE` (`step_import.prefit`).
use super::*;
use std::cell::RefCell;

/// Bodies with fewer faces are finished as before: the helpers' clones and
/// hand-off cost more than a small body's fits.
const MIN_FACES: usize = 64;

fn prefit_mode() -> crate::env_read::Mode {
    #[cfg(not(test))]
    {
        crate::mode_switch!("BREP_STEP_PREFIT", true)
    }
}


/// One coedge's fits as `build_loop` makes them.
struct CoedgeFit {
    edge_id: u64,
    forward: bool,
    curve: NurbsCurve,
    t0: f64,
    t1: f64,
    tolerance: f64,
    ask: Result<crate::PcurveFit, String>,
    /// The floor refit, made exactly when the loop's trigger fires.
    floor: Option<Result<crate::PcurveFit, String>>,
    /// `BREP_STEP_PREFIT_READINGS`: the trim readings of the fit the loop
    /// keeps when no seam stitch or refine changes it (the floor refit when
    /// its residual is the smaller, else the ask fit), with that fit.
    reading: Option<(crate::PcurveFit, Result<super::loops::TrimRead, String>)>,
}

struct FaceFits {
    surface: NurbsSurface,
    coedges: Vec<CoedgeFit>,
    /// `BREP_STEP_ALIAS_VERDICTS`: the face's loops whose aliasing scan read
    /// no aliasing (`builder/alias_verdicts.rs`).
    unaliased: Vec<super::alias_verdicts::Unaliased>,
}

struct Job {
    surface: NurbsSurface,
    tolerance: f64,
    coedges: Vec<(u64, bool, NurbsCurve, f64, f64)>,
    /// The loop reads its trims (`BREP_STEP_PREFIT_READINGS` on and a
    /// resolver or capture that takes them), so the helper reads them too.
    readings: bool,
    /// The face's loops as the aliasing scan reads them, when the switch was
    /// on on the loop's thread; empty otherwise.
    loops: Vec<Vec<super::alias_verdicts::ScanCoedge>>,
}

/// The pending fits of one body; dropping it cancels the jobs not yet started.
pub(super) struct Prefits {
    slots: crate::claim_slots::ClaimSlots<Job, FaceFits>,
    check: bool,
    /// The helpers read their faces' aliasing verdicts.
    verdicts: bool,
}

impl Prefits {
    /// Whether the helpers read aliasing verdicts (`BREP_STEP_ALIAS_VERDICTS`).
    pub(super) fn verdicts(&self) -> bool {
        self.verdicts && !self.check
    }
}

fn fit_job(job: Job) -> FaceFits {
    let Job { surface, tolerance, coedges, readings, loops } = job;
    let fits = coedges
        .into_iter()
        .map(|(edge_id, forward, curve, t0, t1)| {
            // The loop opens a foot memo per coedge around both fits.
            let _feet = crate::pcurve::FootMemoScope::open(&surface);
            let ask = crate::fit_pcurve_on_surface_range(&surface, &curve, t0, t1, forward, tolerance);
            let floor = match &ask {
                Ok(fit) if fit.report.residual > trim_detector_bar(&fit.report)
                    && tolerance > crate::PCURVE_REFINEMENT_TOLERANCE =>
                {
                    Some(crate::fit_pcurve_on_surface_range(
                        &surface, &curve, t0, t1, forward, crate::PCURVE_REFINEMENT_TOLERANCE,
                    ))
                }
                _ => None,
            };
            // The fit `build_loop` keeps: the floor refit replaces the ask fit
            // only on a strictly smaller residual (an error in either leaves
            // the loop to report it).
            let kept = match (&ask, &floor) {
                (Ok(ask), None) => Some(ask),
                (Ok(ask), Some(Ok(floor))) => Some(if floor.report.residual < ask.report.residual { floor } else { ask }),
                _ => None,
            };
            let reading = kept.filter(|_| readings).map(|fit| {
                (fit.clone(), super::loops::read_trim(&surface, &curve, t0, t1, forward, fit))
            });
            CoedgeFit { edge_id, forward, curve, t0, t1, tolerance, ask, floor, reading }
        })
        .collect();
    let unaliased = if loops.is_empty() { Vec::new() } else { super::alias_verdicts::helper_verdicts(&surface, loops) };
    FaceFits { surface, coedges: fits, unaliased }
}

impl<'a> SolidBuilder<'a> {
    /// Spawn the face jobs for `pending` when the switch is on and the body is
    /// large ([`crate::claim_slots`]); `None` otherwise (and always without
    /// the `parallel` feature).
    pub(super) fn spawn_prefits(&self, pending: &[PendingFace]) -> Result<Option<Prefits>, String> {
        let mode = prefit_mode();
        if mode == crate::env_read::Mode::Off
            || pending.len() < MIN_FACES
            || !crate::claim_slots::helpers_available()
        {
            return Ok(None);
        }
        // A memo already open on this thread could answer the loop's fits
        // from feet a helper does not see.
        if crate::pcurve::foot_memo_open() {
            return Ok(None);
        }
        // `BREP_STEP_ALIAS_VERDICTS`, read here on the loop's thread; never
        // under the joint pass's trace, which runs every scan live.
        let verdicts = super::alias_verdicts::alias_verdicts_enabled() && std::env::var("BREP_DEBUG_PCURVE").is_err();
        let readings = prefit_readings_enabled() && (self.resolver.trim_readings || self.readings.is_some());
        let mut jobs = Vec::with_capacity(pending.len());
        for face in pending {
            let mut coedges = Vec::new();
            let mut loops = Vec::new();
            for (specs, _) in &face.bounds {
                for &(edge_id, forward) in specs {
                    let edge = self.edge_record(edge_id);
                    coedges.push((edge_id, forward, edge.curve.clone(), edge.t0, edge.t1));
                }
                if verdicts {
                    loops.push(
                        specs
                            .iter()
                            .map(|&(edge_id, forward)| {
                                let edge = self.edge_record(edge_id);
                                super::alias_verdicts::ScanCoedge {
                                    curve: edge.curve.clone(),
                                    t0: edge.t0,
                                    t1: edge.t1,
                                    forward,
                                    degenerate: edge.degenerate,
                                }
                            })
                            .collect(),
                    );
                }
            }
            jobs.push(Job { surface: face.surface.clone(), tolerance: loop_pcurve_tolerance(&face.surface)?, coedges, readings, loops });
        }
        let Some(slots) = crate::claim_slots::ClaimSlots::spawn(jobs, fit_job) else {
            return Ok(None);
        };
        STATS.with(|stats| stats.take());
        Ok(Some(Prefits { slots, check: mode == crate::env_read::Mode::Check, verdicts }))
    }
}

impl Prefits {
    /// Take face `index`'s fits if a helper finished them; otherwise claim the
    /// face so no helper spends time on it. Never waits.
    pub(super) fn claim(&mut self, index: usize) -> Installed {
        let fits = self.slots.claim(index);
        Installed::install(fits, self.check)
    }

    pub(super) fn report(&self, shell_ref: usize) {
        let stats = STATS.with(|stats| stats.take());
        if crate::profile::profile_enabled() {
            profile_line!(
                "step_import.prefit shell=#{shell_ref} faces={} check={} taken={} claimed={} hits={} misses={} unused={} disagree={} hit_fit_ms={:.2} reading_hits={} reading_disagree={}",
                self.slots.len(),
                u8::from(self.check),
                self.slots.taken,
                self.slots.claimed,
                stats.hits,
                stats.misses,
                stats.unused,
                stats.disagree,
                stats.hit_fit_ms,
                stats.reading_hits,
                stats.reading_disagree
            );
        }
    }
}

#[derive(Default)]
struct Stats {
    hits: usize,
    misses: usize,
    unused: usize,
    disagree: usize,
    hit_fit_ms: f64,
    /// Trim readings taken from a helper, and (under `check`) found to differ.
    reading_hits: usize,
    reading_disagree: usize,
}

thread_local! {
    /// The fits offered to the face being finished on this thread.
    static OFFERED: RefCell<Option<(FaceFits, Vec<bool>, bool)>> = const { RefCell::new(None) };
    static STATS: RefCell<Stats> = RefCell::new(Stats::default());
}

/// The face's offered fits, withdrawn (and its unused ones counted) on drop.
pub(super) struct Installed(Option<Option<(FaceFits, Vec<bool>, bool)>>);

impl Installed {
    fn install(fits: Option<FaceFits>, check: bool) -> Self {
        let offered = fits.map(|mut fits| {
            // A helper's verdicts join the body's memo (`check` reads every
            // scan live instead, as it fits every coedge live).
            for entry in std::mem::take(&mut fits.unaliased) {
                if !check {
                    super::alias_verdicts::record_unaliased(entry);
                }
            }
            let used = vec![false; fits.coedges.len()];
            (fits, used, check)
        });
        Self(Some(OFFERED.with(|cell| cell.replace(offered))))
    }
}

impl Drop for Installed {
    fn drop(&mut self) {
        let previous = self.0.take().flatten();
        let withdrawn = OFFERED.with(|cell| cell.replace(previous));
        if let Some((_, used, _)) = withdrawn {
            let unused = used.iter().filter(|used| !**used).count();
            STATS.with(|stats| stats.borrow_mut().unused += unused);
        }
    }
}

fn same_floats(a: &[f64], b: &[f64]) -> bool {
    a.len() == b.len() && a.iter().zip(b).all(|(x, y)| x.to_bits() == y.to_bits())
}

fn same_point(a: &crate::Vec4, b: &crate::Vec4) -> bool {
    same_floats(&[a.x, a.y, a.z, a.w], &[b.x, b.y, b.z, b.w])
}

pub(super) fn same_curve(a: &NurbsCurve, b: &NurbsCurve) -> bool {
    a.degree == b.degree
        && same_floats(&a.knots, &b.knots)
        && a.control_points.len() == b.control_points.len()
        && a.control_points.iter().zip(&b.control_points).all(|(p, q)| same_point(p, q))
}

pub(super) fn same_surface(a: &NurbsSurface, b: &NurbsSurface) -> bool {
    a.degree_u == b.degree_u
        && a.degree_v == b.degree_v
        && same_floats(&a.knots_u, &b.knots_u)
        && same_floats(&a.knots_v, &b.knots_v)
        && a.control_points.len() == b.control_points.len()
        && a.control_points.iter().zip(&b.control_points).all(|(r, s)| {
            r.len() == s.len() && r.iter().zip(s).all(|(p, q)| same_point(p, q))
        })
}

fn same_fit(a: &Result<crate::PcurveFit, String>, b: &Result<crate::PcurveFit, String>) -> bool {
    match (a, b) {
        (Ok(a), Ok(b)) => same_curve(&a.curve, &b.curve) && format!("{:?}", a.report) == format!("{:?}", b.report),
        (Err(a), Err(b)) => a == b,
        _ => false,
    }
}

/// `fit_pcurve_on_surface_range(surface, edge.curve, edge.t0, edge.t1,
/// forward, tolerance)` as `build_loop` asks for it: from the face's offered
/// fits when one matches every input bit for bit, else fitted here.
pub(super) fn fit_range(
    surface: &NurbsSurface,
    edge: &EdgeRecord,
    forward: bool,
    tolerance: f64,
) -> Result<crate::PcurveFit, String> {
    let fresh = || crate::fit_pcurve_on_surface_range(surface, &edge.curve, edge.t0, edge.t1, forward, tolerance);
    let offered = OFFERED.with(|cell| {
        let mut cell = cell.borrow_mut();
        let (fits, used, check) = cell.as_mut()?;
        if !same_surface(&fits.surface, surface) {
            return None;
        }
        let index = fits.coedges.iter().position(|fit| {
            fit.edge_id == edge.id
                && fit.forward == forward
                && fit.t0.to_bits() == edge.t0.to_bits()
                && fit.t1.to_bits() == edge.t1.to_bits()
                && same_curve(&fit.curve, &edge.curve)
                && (fit.tolerance.to_bits() == tolerance.to_bits()
                    || (fit.floor.is_some() && tolerance.to_bits() == crate::PCURVE_REFINEMENT_TOLERANCE.to_bits()))
        })?;
        used[index] = true;
        let fit = &fits.coedges[index];
        let result = if fit.tolerance.to_bits() == tolerance.to_bits() { fit.ask.clone() } else { fit.floor.clone()? };
        Some((result, *check))
    });
    let Some((result, check)) = offered else {
        if OFFERED.with(|cell| cell.borrow().is_some()) {
            STATS.with(|stats| stats.borrow_mut().misses += 1);
        }
        return fresh();
    };
    if !check {
        STATS.with(|stats| stats.borrow_mut().hits += 1);
        return result;
    }
    let started = web_time::Instant::now();
    let computed = fresh();
    let elapsed = started.elapsed().as_secs_f64() * 1_000.0;
    let agrees = same_fit(&result, &computed);
    STATS.with(|stats| {
        let mut stats = stats.borrow_mut();
        stats.hits += 1;
        stats.hit_fit_ms += elapsed;
        stats.disagree += usize::from(!agrees);
    });
    computed
}

/// `BREP_STEP_PREFIT_READINGS` (default on; `=0` reads every trim on the
/// loop's thread): a prefit helper also reads the trim it fitted
/// ([`super::loops::read_trim`]), so the loop's report block takes the reading
/// instead of making it. With the fits moved onto helpers, these reads were
/// the largest part of a 496-face SolidWorks part's face loop left on one
/// thread (13.7 of 25.8 s, mostly `image_to_foot`'s 1025 global projections
/// per trim off its bar). A reading is taken only for the very fit the loop
/// keeps, compared bit for bit, so it is the reading the loop would make;
/// under `BREP_STEP_PREFIT=check` the loop reads every trim itself and counts
/// any difference (`reading_disagree`).
fn prefit_readings_enabled() -> bool {
    crate::switch!("BREP_STEP_PREFIT_READINGS", true)
}

/// [`super::loops::read_trim`] of `fit` as the trim of `edge` in sense
/// `forward` on `surface`: from the face's offered readings when a helper read
/// this fit for these inputs (every bit the same), else read here.
pub(super) fn read_trim_range(
    surface: &NurbsSurface,
    edge: &EdgeRecord,
    forward: bool,
    fit: &crate::PcurveFit,
) -> Result<super::loops::TrimRead, String> {
    let fresh = || super::loops::read_trim(surface, &edge.curve, edge.t0, edge.t1, forward, fit);
    let offered = OFFERED.with(|cell| {
        let cell = cell.borrow();
        let (fits, _, check) = cell.as_ref()?;
        if !same_surface(&fits.surface, surface) {
            return None;
        }
        let coedge = fits.coedges.iter().find(|coedge| {
            coedge.edge_id == edge.id
                && coedge.forward == forward
                && coedge.t0.to_bits() == edge.t0.to_bits()
                && coedge.t1.to_bits() == edge.t1.to_bits()
                && same_curve(&coedge.curve, &edge.curve)
        })?;
        let (read_fit, reading) = coedge.reading.as_ref()?;
        let same = same_curve(&read_fit.curve, &fit.curve) && format!("{:?}", read_fit.report) == format!("{:?}", fit.report);
        same.then(|| (reading.clone(), *check))
    });
    let Some((reading, check)) = offered else {
        return fresh();
    };
    if !check {
        STATS.with(|stats| stats.borrow_mut().reading_hits += 1);
        return reading;
    }
    let computed = fresh();
    let agrees = format!("{reading:?}") == format!("{computed:?}");
    STATS.with(|stats| {
        let mut stats = stats.borrow_mut();
        stats.reading_hits += 1;
        stats.reading_disagree += usize::from(!agrees);
    });
    computed
}
