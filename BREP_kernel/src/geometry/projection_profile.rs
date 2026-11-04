//! PROJECTION PROFILE (2026-11-02): under `BREP_PROFILE`, the
//! calls and wall time of every surface projection and inversion entry, the
//! marched surface-surface intersections, and the point/derivative
//! evaluation counts, each by the carrier's class, so a model's time on
//! affinely scaled spheres (not recognized analytic carriers, so every
//! projection on them ran the general grid-seeded Newton) can be read per
//! model. Off: one static read per hooked call, nothing else.
//!
//! Lanes overlap (a global projection evaluates the surface; an inversion
//! calls the projector), so each lane is INCLUSIVE; the `outermost` rows count
//! only the outermost active lap per thread and partition the projection time
//! by class without overlap. Evaluation lanes count calls only (no clock), so
//! the profile run's projection times are not inflated by the instrument.
//! Printed by `examples/history_timing_probe.rs` after the run as
//! `projection.profile lane=<lane> class=<class> calls=<n> ms=<ms>` lines.

use crate::NurbsSurface;
use std::cell::Cell;
use std::sync::atomic::{AtomicU64, Ordering};
use web_time::Instant;

/// 0 recognized analytic (plane, cylinder, cone, sphere, torus, revolution),
/// 1 affine sphere (a sphere net under a non-similarity affine map),
/// 2 every other general patch.
pub(crate) const CLASSES: usize = 3;
const CLASS_NAMES: [&str; CLASSES] = ["analytic", "affine_sphere", "general"];

pub(crate) const LANE_PROJECT: usize = 0;
pub(crate) const LANE_SEEDED: usize = 1;
pub(crate) const LANE_GENERAL: usize = 2;
pub(crate) const LANE_FROM_SEED: usize = 3;
pub(crate) const LANE_FROM_NEAREST_SEED: usize = 4;
pub(crate) const LANE_INVERT_CHECKED: usize = 5;
pub(crate) const LANE_INTERSECT_SURFACES: usize = 6;
pub(crate) const LANE_OUTERMOST: usize = 7;
pub(crate) const LANE_EVALUATE: usize = 8;
pub(crate) const LANE_DERIVATIVES: usize = 9;
pub(crate) const LANE_INTERSECT_CURVE_SURFACE: usize = 10;
const LANES: usize = 11;
const LANE_NAMES: [&str; LANES] = [
    "project_point_to_surface",
    "project_point_to_surface_seeded",
    "project_point_to_surface_general",
    "project_point_to_surface_from_seed",
    "project_point_to_surface_from_nearest_seed",
    "invert_checked_uncached",
    "intersect_surfaces",
    "outermost",
    "evaluate_homogeneous",
    "derivatives",
    "intersect_curve_surface",
];

static CALLS: [[AtomicU64; CLASSES]; LANES] = [const { [const { AtomicU64::new(0) }; CLASSES] }; LANES];
static NANOS: [[AtomicU64; CLASSES]; LANES] = [const { [const { AtomicU64::new(0) }; CLASSES] }; LANES];

thread_local! { static DEPTH: Cell<u32> = const { Cell::new(0) }; }

/// The carrier's class for this profile. An affine sphere is read by shape
/// (the full sphere net: degree (2, 2), 9 x 5 control points) when it is not a
/// recognized analytic carrier; the recognizer itself is the switch's.
pub(crate) fn class(surface: &NurbsSurface) -> usize {
    if surface.analytic().is_some() {
        0
    } else if crate::affine_sphere::has_sphere_net_shape(surface) {
        1
    } else {
        2
    }
}

/// Class of a surface pair: an affine sphere on either side names the pair,
/// otherwise analytic only when both are.
fn pair_class(first: &NurbsSurface, second: &NurbsSurface) -> usize {
    let (a, b) = (class(first), class(second));
    if a == 1 || b == 1 {
        1
    } else {
        a.max(b)
    }
}

pub(crate) struct Lap(Option<(Instant, usize, usize, bool)>);

impl Lap {
    pub(crate) fn enter(lane: usize, surface: &NurbsSurface) -> Self {
        if !crate::profile::profile_enabled() {
            return Self(None);
        }
        Self::begin(lane, class(surface))
    }

    pub(crate) fn enter_pair(first: &NurbsSurface, second: &NurbsSurface) -> Self {
        if !crate::profile::profile_enabled() {
            return Self(None);
        }
        Self::begin(LANE_INTERSECT_SURFACES, pair_class(first, second))
    }

    fn begin(lane: usize, class: usize) -> Self {
        let outermost = DEPTH.with(|depth| {
            let value = depth.get();
            depth.set(value + 1);
            value == 0
        });
        Self(Some((Instant::now(), lane, class, outermost)))
    }
}

impl Drop for Lap {
    fn drop(&mut self) {
        if let Some((start, lane, class, outermost)) = self.0 {
            let nanos = start.elapsed().as_nanos() as u64;
            CALLS[lane][class].fetch_add(1, Ordering::Relaxed);
            NANOS[lane][class].fetch_add(nanos, Ordering::Relaxed);
            if outermost {
                CALLS[LANE_OUTERMOST][class].fetch_add(1, Ordering::Relaxed);
                NANOS[LANE_OUTERMOST][class].fetch_add(nanos, Ordering::Relaxed);
            }
            DEPTH.with(|depth| depth.set(depth.get() - 1));
        }
    }
}

/// Count one call of a lane that is not timed (evaluation).
#[inline]
pub(crate) fn count(lane: usize, surface: &NurbsSurface) {
    if crate::profile::profile_enabled() {
        CALLS[lane][class(surface)].fetch_add(1, Ordering::Relaxed);
    }
}

/// The accumulated rows, one line each, and reset.
pub fn report() -> String {
    let mut out = String::new();
    for lane in 0..LANES {
        for class in 0..CLASSES {
            let calls = CALLS[lane][class].swap(0, Ordering::Relaxed);
            let nanos = NANOS[lane][class].swap(0, Ordering::Relaxed);
            if calls == 0 {
                continue;
            }
            out.push_str(&format!(
                "projection.profile lane={} class={} calls={calls} ms={:.3}\n",
                LANE_NAMES[lane],
                CLASS_NAMES[class],
                nanos as f64 / 1e6
            ));
        }
    }
    out
}
