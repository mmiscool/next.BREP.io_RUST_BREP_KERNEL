//! `BREP_STEP_ALIAS_VERDICTS` (default on; `=0` scans every loop on the face
//! loop's thread as before): under the `parallel` feature,
//! the joint branch pass's aliasing scan is read on the prefit helpers.
//!
//! `joint_branch_pcurves` first scans every loop of a general open carrier for
//! the wrap-aliasing signature (`aliasing_scan`): 65 single-seed inversions per
//! coedge. On a large surface model nearly every face is such a loop and
//! nearly none aliases, so the scan's answer is "no" and the pass returns
//! `None`; on a 4236-face body that was a quarter of the face loop's thread.
//! The answer is a function of the carrier and the loop's edges alone (their
//! curves, ranges and senses).
//!
//! With the switch on, the prefit helpers (`builder/prefit.rs`) scan their
//! faces' loops ahead of the face loop, and a face's loops that read no
//! aliasing are taken with its fits into this body's memo, keyed by every
//! input bit for bit. The face loop answers such a loop from the memo; a loop
//! whose inputs changed since (an edge stitched or replaced) misses and is
//! scanned as before. Only "no aliasing" is kept: an aliased loop is scanned
//! and assigned as before, every time. Without helpers (no `parallel`
//! feature, a small body, the prefit off or in `check` mode) no memo opens and
//! nothing changes. Under `BREP_DEBUG_PCURVE` every scan runs, so its trace
//! reads as before.
use super::*;
use std::cell::RefCell;
use std::hash::{Hash, Hasher};

/// The switch, read on the thread that builds the body when it spawns the
/// helpers (they never read the environment or a test override).
pub(super) fn alias_verdicts_enabled() -> bool {
    crate::switch!("BREP_STEP_ALIAS_VERDICTS", true)
}

/// One coedge of a loop, as the scan reads it: the edge's curve over its
/// range, in the coedge's sense, and whether the edge is degenerate.
#[derive(Clone)]
pub(super) struct ScanCoedge {
    pub(super) curve: NurbsCurve,
    pub(super) t0: f64,
    pub(super) t1: f64,
    pub(super) forward: bool,
    pub(super) degenerate: bool,
}

/// A loop whose scan read no aliasing, by its inputs.
pub(super) struct Unaliased {
    hash: u64,
    surface: NurbsSurface,
    coedges: Vec<ScanCoedge>,
}

impl Unaliased {
    pub(super) fn new(surface: &NurbsSurface, coedges: Vec<ScanCoedge>) -> Self {
        Self { hash: inputs_hash(surface, &coedges), surface: surface.clone(), coedges }
    }
}

fn inputs_hash(surface: &NurbsSurface, coedges: &[ScanCoedge]) -> u64 {
    let mut hasher = rustc_hash::FxHasher::default();
    surface.degree_u.hash(&mut hasher);
    surface.degree_v.hash(&mut hasher);
    for knot in surface.knots_u.iter().chain(&surface.knots_v) {
        knot.to_bits().hash(&mut hasher);
    }
    for point in surface.control_points.iter().flatten() {
        for value in [point.x, point.y, point.z, point.w] {
            value.to_bits().hash(&mut hasher);
        }
    }
    for coedge in coedges {
        coedge.curve.degree.hash(&mut hasher);
        for knot in &coedge.curve.knots {
            knot.to_bits().hash(&mut hasher);
        }
        for point in &coedge.curve.control_points {
            for value in [point.x, point.y, point.z, point.w] {
                value.to_bits().hash(&mut hasher);
            }
        }
        coedge.t0.to_bits().hash(&mut hasher);
        coedge.t1.to_bits().hash(&mut hasher);
        coedge.forward.hash(&mut hasher);
        coedge.degenerate.hash(&mut hasher);
    }
    hasher.finish()
}

fn same_inputs(entry: &Unaliased, surface: &NurbsSurface, coedges: &[ScanCoedge]) -> bool {
    super::prefit::same_surface(&entry.surface, surface)
        && entry.coedges.len() == coedges.len()
        && entry.coedges.iter().zip(coedges).all(|(a, b)| {
            a.forward == b.forward
                && a.degenerate == b.degenerate
                && a.t0.to_bits() == b.t0.to_bits()
                && a.t1.to_bits() == b.t1.to_bits()
                && super::prefit::same_curve(&a.curve, &b.curve)
        })
}

#[derive(Default)]
struct Memo {
    loops: HashMap<u64, Vec<Unaliased>>,
    /// Scans answered from the memo, and verdicts taken from helpers.
    hits: usize,
    offered: usize,
}

thread_local! {
    /// `None` outside a body build with the switch on.
    static MEMO: RefCell<Option<Memo>> = const { RefCell::new(None) };
}

/// The memo of one body's build, open while it lives (when the switch is on).
pub(super) struct MemoScope(Option<Option<Memo>>);

impl MemoScope {
    pub(super) fn open(enabled: bool) -> Self {
        if !enabled {
            return Self(None);
        }
        Self(Some(MEMO.with(|cell| cell.replace(Some(Memo::default())))))
    }

    pub(super) fn report(&self, shell_ref: usize) {
        if self.0.is_none() || !crate::profile::profile_enabled() {
            return;
        }
        MEMO.with(|cell| {
            if let Some(memo) = cell.borrow().as_ref() {
                profile_line!(
                    "step_import.alias_verdicts shell=#{shell_ref} hits={} offered={}",
                    memo.hits,
                    memo.offered
                );
            }
        });
    }
}

impl Drop for MemoScope {
    fn drop(&mut self) {
        if let Some(previous) = self.0.take() {
            let closed = MEMO.with(|cell| cell.replace(previous));
            drop(closed);
        }
    }
}



/// Whether this loop's scan already read no aliasing (counted as a hit).
pub(super) fn known_unaliased(surface: &NurbsSurface, coedges: &[ScanCoedge]) -> bool {
    MEMO.with(|cell| {
        let mut cell = cell.borrow_mut();
        let Some(memo) = cell.as_mut() else {
            return false;
        };
        let hash = inputs_hash(surface, coedges);
        let found = memo
            .loops
            .get(&hash)
            .is_some_and(|entries| entries.iter().any(|entry| same_inputs(entry, surface, coedges)));
        memo.hits += usize::from(found);
        found
    })
}

/// Keep a loop a helper's scan read no aliasing on.
pub(super) fn record_unaliased(entry: Unaliased) {
    MEMO.with(|cell| {
        if let Some(memo) = cell.borrow_mut().as_mut() {
            memo.offered += 1;
            memo.loops.entry(entry.hash).or_default().push(entry);
        }
    });
}

/// The structural class the joint pass is for: a general carrier, open in
/// both directions. `false` for every other carrier (no scan is made).
pub(super) fn scanned_class(surface: &NurbsSurface) -> Result<bool, String> {
    let (closed_u, closed_v) = surface.closed_directions()?;
    Ok(!(closed_u || closed_v || surface.analytic().is_some() || surface.is_affine()?))
}

/// Samples per coedge of the scan and of the joint pass.
pub(super) const SCAN_SAMPLES: usize = 64;

/// Each coedge's 3D samples in coedge sense, or `None` when one is degenerate
/// (a collapsed pole has no branch to track).
pub(super) fn loop_samples(coedges: &[ScanCoedge]) -> Result<Option<Vec<Vec<Vec3>>>, String> {
    let mut samples = Vec::with_capacity(coedges.len());
    for coedge in coedges {
        if coedge.degenerate {
            return Ok(None);
        }
        let mut points = Vec::with_capacity(SCAN_SAMPLES + 1);
        for k in 0..=SCAN_SAMPLES {
            let f = k as f64 / SCAN_SAMPLES as f64;
            let edge_fraction = if coedge.forward { f } else { 1.0 - f };
            points.push(coedge.curve.evaluate(coedge.t0 + (coedge.t1 - coedge.t0) * edge_fraction)?);
        }
        samples.push(points);
    }
    Ok(Some(samples))
}

/// The wrap-aliasing signature (see `joint_branch_pcurves`): whether the
/// single-seed inversion of consecutive samples jumps by more than a fifth of
/// the normalised domain, and the worst distance it reports (trace only).
pub(super) fn aliasing_scan(surface: &NurbsSurface, samples: &[Vec<Vec3>]) -> Result<(bool, f64), String> {
    const JUMP: f64 = 0.2;
    let [u0, u1] = surface.domain_u()?;
    let [v0, v1] = surface.domain_v()?;
    let u_span = (u1 - u0).abs().max(1e-12);
    let v_span = (v1 - v0).abs().max(1e-12);
    let norm_step = |a: (f64, f64), b: (f64, f64)| -> f64 {
        (((a.0 - b.0) / u_span).powi(2) + ((a.1 - b.1) / v_span).powi(2)).sqrt()
    };
    let mut aliased = false;
    let mut global_residual = 0.0f64;
    for pts in samples {
        let mut prev: Option<(f64, f64)> = None;
        for p in pts {
            let g = crate::projection::project_point_to_surface_from_nearest_seed(surface, *p)?;
            global_residual = global_residual.max(g.distance);
            if let Some(pv) = prev {
                if norm_step((g.u, g.v), pv) > JUMP {
                    aliased = true;
                }
            }
            prev = Some((g.u, g.v));
        }
    }
    Ok((aliased, global_residual))
}

/// A helper's reading of one face's loops: the loops whose scan reads no
/// aliasing (loops outside the class, or with a degenerate edge, are not
/// scanned and not kept). An error in any reading drops that loop.
pub(super) fn helper_verdicts(surface: &NurbsSurface, loops: Vec<Vec<ScanCoedge>>) -> Vec<Unaliased> {
    if !scanned_class(surface).unwrap_or(false) {
        return Vec::new();
    }
    let mut kept = Vec::new();
    for coedges in loops {
        if coedges.len() < 2 {
            continue;
        }
        let Ok(Some(samples)) = loop_samples(&coedges) else {
            continue;
        };
        if let Ok((false, _)) = aliasing_scan(surface, &samples) {
            kept.push(Unaliased::new(surface, coedges));
        }
    }
    kept
}

