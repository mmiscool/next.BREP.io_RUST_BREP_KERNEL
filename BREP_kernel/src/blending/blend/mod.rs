//! General rolling-ball blends (Golovanov §4.9 + §6.9 surgery).
//!
//! The blend surface is the rational surface (4.9.1) built from three
//! curves: the two support curves cr(t) ⊂ F1 and cs(t) ⊂ F2 where a
//! sphere of radius ρ touches both faces, and the middle curve at the
//! intersection of the two tangent planes, weighted w(t) = cos(α/2)
//! (4.9.3).  The tangency system (4.9.2) + the section-plane constraint
//! (4.9.5) — the sphere center pinned to the normal plane of the edge —
//! is solved per station by Newton with a finite-difference Jacobian.
//!
//! Topology is rebuilt the §6.9 way, with NO tool solid and NO boolean:
//! each mating face's loop swaps the blended edge's coedge for a coedge
//! on its support curve (trimming the adjacent seam edge when the face
//! is a closed carrier with seam-in-one-loop topology), and the blend
//! face is inserted with its own seam.  Open edges take the same route
//! (`edge/open.rs`), and a whole SELECTION goes through `network.rs`, which
//! marches every stripe against the original solid and solves each shared
//! corner from its ball before anything is cut.

mod carve;
mod chain;
mod collapse;
mod corner;
mod edge;
mod fold;
mod miter;
mod network;
mod planar_chart;
mod restrict;
mod runout;
mod stations;
mod rows;
mod track_fit;

thread_local! {
    static REFINE_OFF_FOR_TEST: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
}

/// Whether the open march's local station refinement runs. Escape hatch
/// `BREP_BLEND_REFINE=0` restores the uniform ladder exactly; a test turns it
/// off for its own thread with `set_refinement_off_for_test`.
pub(crate) fn station_refinement_on() -> bool {
    std::env::var("BREP_BLEND_REFINE").as_deref() != Ok("0")
        && !REFINE_OFF_FOR_TEST.with(|off| off.get())
}

thread_local! {
    static BLEND_NOTES: std::cell::RefCell<Vec<String>> = const { std::cell::RefCell::new(Vec::new()) };
}

/// Record what a blend that SHIPS needs its feature to say about it — today,
/// a march whose rails still stand off their carriers after its station
/// ladder and local refinement ran out. Drained per feature by the history
/// loop ([`take_blend_notes`]), beside the soundness acceptance's repairs.
pub(crate) fn record_blend_note(note: String) {
    BLEND_NOTES.with(|notes| notes.borrow_mut().push(note));
}

/// Take every blend note recorded on this thread since the last call.
/// Blending runs on the feature's own thread (no parallel march), so the
/// history loop's drain sees every note its feature recorded.
pub fn take_blend_notes() -> Vec<String> {
    BLEND_NOTES.with(|notes| std::mem::take(&mut *notes.borrow_mut()))
}

/// The ledger's length, and a way back to it: a lane that refuses takes back
/// the notes it recorded, so a note never describes a wall that did not ship.
pub(crate) fn blend_notes_mark() -> usize {
    BLEND_NOTES.with(|notes| notes.borrow().len())
}

pub(crate) fn blend_notes_truncate(mark: usize) {
    BLEND_NOTES.with(|notes| notes.borrow_mut().truncate(mark));
}


pub use chain::{blend_smooth_chain, blend_smooth_chain_if_closed};
pub(crate) use corner::round_concave_chain_corner;
pub use corner::round_convex_corner;
pub use edge::{blend_closed_edge, blend_edge_variable, blend_open_edge};
pub(crate) use collapse::is_full_width_collapse;
pub(crate) use planar_chart::{
    fit_planar_charts_to_trims, is_planar_chart_refusal,
};
pub(crate) use track_fit::PCURVE_OFF_FLOOR;
pub(crate) use edge::{consumed_band, is_consumed_snap, is_rail_collapse};
pub(crate) use network::{
    blend_star_network, degenerate_corner_setbacks, mixed_convexity_corner, DegenerateSetback,
    MixedCorner,
};
pub(crate) use miter::is_marched_fit_off_carriers;
pub(crate) use runout::{plan_runout, RunoutPlan};
pub(crate) use fold::{is_wall_fold, WALL_FOLDS};
pub(crate) use stations::is_ball_off_carrier;
