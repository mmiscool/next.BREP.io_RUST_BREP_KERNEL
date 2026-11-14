//! `BREP_STEP_PREFIT` (default off; `1` on, `check` compares): the projection
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
        crate::mode_switch!("BREP_STEP_PREFIT", false)
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
}

struct FaceFits {
    surface: NurbsSurface,
    coedges: Vec<CoedgeFit>,
}

struct Job {
    surface: NurbsSurface,
    tolerance: f64,
    coedges: Vec<(u64, bool, NurbsCurve, f64, f64)>,
}

/// The pending fits of one body; dropping it cancels the jobs not yet started.
pub(super) struct Prefits {
    slots: crate::claim_slots::ClaimSlots<Job, FaceFits>,
    check: bool,
}

fn fit_job(job: Job) -> FaceFits {
    let Job { surface, tolerance, coedges } = job;
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
            CoedgeFit { edge_id, forward, curve, t0, t1, tolerance, ask, floor }
        })
        .collect();
    FaceFits { surface, coedges: fits }
}

impl<'a> SolidBuilder<'a> {
    /// Spawn the face jobs for `pending` when the switch is on and the body is
    /// large ([`crate::claim_slots`]); `None` otherwise (and always without
    /// the `parallel` feature).
    pub(super) fn spawn_prefits(&self, pending: &[PendingFace]) -> Result<Option<Prefits>, String> {
        let mode = prefit_mode();
        if mode == crate::env_read::Mode::Off || pending.len() < MIN_FACES {
            return Ok(None);
        }
        // A memo already open on this thread could answer the loop's fits
        // from feet a helper does not see.
        if crate::pcurve::foot_memo_open() {
            return Ok(None);
        }
        let mut jobs = Vec::with_capacity(pending.len());
        for face in pending {
            let mut coedges = Vec::new();
            for (specs, _) in &face.bounds {
                for &(edge_id, forward) in specs {
                    let edge = self.edge_record(edge_id);
                    coedges.push((edge_id, forward, edge.curve.clone(), edge.t0, edge.t1));
                }
            }
            jobs.push(Job { surface: face.surface.clone(), tolerance: loop_pcurve_tolerance(&face.surface)?, coedges });
        }
        let Some(slots) = crate::claim_slots::ClaimSlots::spawn(jobs, fit_job) else {
            return Ok(None);
        };
        STATS.with(|stats| stats.take());
        Ok(Some(Prefits { slots, check: mode == crate::env_read::Mode::Check }))
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
                "step_import.prefit shell=#{shell_ref} faces={} check={} taken={} claimed={} hits={} misses={} unused={} disagree={} hit_fit_ms={:.2}",
                self.slots.len(),
                u8::from(self.check),
                self.slots.taken,
                self.slots.claimed,
                stats.hits,
                stats.misses,
                stats.unused,
                stats.disagree,
                stats.hit_fit_ms
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
        let offered = fits.map(|fits| {
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

fn same_curve(a: &NurbsCurve, b: &NurbsCurve) -> bool {
    a.degree == b.degree
        && same_floats(&a.knots, &b.knots)
        && a.control_points.len() == b.control_points.len()
        && a.control_points.iter().zip(&b.control_points).all(|(p, q)| same_point(p, q))
}

fn same_surface(a: &NurbsSurface, b: &NurbsSurface) -> bool {
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
