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
pub(crate) mod runout;
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

/// A fillet chain wall that SHIPPED short of the construction request its
/// accepted rung made (`KernelTolerances::model` on the wall between its
/// rails): which wall — the blend face's name and its exact surface — what
/// was asked, what the wall reads, and the typed budget that ran out.
#[derive(Clone, Debug)]
pub(crate) struct WallModelReport {
    pub(crate) name: Option<String>,
    pub(crate) surface: crate::NurbsSurface,
    pub(crate) request: f64,
    pub(crate) residual: f64,
    /// What else the lane measured that the message must say (an unaccepted
    /// rung's deficit, a ladder rung's local refinement), verbatim.
    pub(crate) detail: Option<String>,
    pub(crate) budget: crate::ApproximationBudget,
}

thread_local! {
    static BLEND_APPROXIMATIONS: std::cell::RefCell<Vec<WallModelReport>> = const { std::cell::RefCell::new(Vec::new()) };
}

thread_local! {
    static BLEND_OPERATION_DEPTH: std::cell::Cell<usize> = const { std::cell::Cell::new(0) };
}

/// One blend OPERATION's scope on this thread: every public blend entry point
/// (and the history loop, around each feature) holds one. The OUTERMOST
/// entry drops whatever an earlier operation recorded and nobody consumed —
/// a direct API caller consumes ([`take_blend_approximations`]) after the
/// operation whose reports it wants, before starting the next — while a
/// nested entry (a fillet's per-edge chain blend) clears nothing, so every
/// report of the operation in flight is kept, however many there are.
pub(crate) struct BlendOperation;

impl BlendOperation {
    pub(crate) fn enter() -> BlendOperation {
        BLEND_OPERATION_DEPTH.with(|depth| {
            if depth.get() == 0 {
                BLEND_APPROXIMATIONS.with(|reports| reports.borrow_mut().clear());
            }
            depth.set(depth.get() + 1);
        });
        BlendOperation
    }
}

impl Drop for BlendOperation {
    fn drop(&mut self) {
        BLEND_OPERATION_DEPTH.with(|depth| depth.set(depth.get().saturating_sub(1)));
    }
}

/// Record a wall that ships with its request unmet. Written by the rung that
/// keeps its body; whether the body reaches a caller is decided at
/// consumption ([`take_blend_approximations`]), not here. Every report of
/// the operation in flight is kept (see [`BlendOperation`]).
pub(crate) fn record_wall_model_report(report: WallModelReport) {
    BLEND_APPROXIMATIONS.with(|reports| reports.borrow_mut().push(report));
}


/// Whether any wall-model report is waiting (the history loop reads the
/// returned bodies only then).
pub(crate) fn blend_approximations_pending() -> bool {
    BLEND_APPROXIMATIONS.with(|reports| !reports.borrow().is_empty())
}

/// Bit-identical surfaces: the same degrees, knots and homogeneous control
/// points, compared by their bits.
pub(crate) fn same_surface(a: &crate::NurbsSurface, b: &crate::NurbsSurface) -> bool {
    let bits = |values: &[f64]| values.iter().map(|value| value.to_bits()).collect::<Vec<_>>();
    a.degree_u == b.degree_u
        && a.degree_v == b.degree_v
        && bits(&a.knots_u) == bits(&b.knots_u)
        && bits(&a.knots_v) == bits(&b.knots_v)
        && a.control_points.len() == b.control_points.len()
        && a.control_points.iter().zip(&b.control_points).all(|(row_a, row_b)| {
            row_a.len() == row_b.len()
                && row_a.iter().zip(row_b).all(|(p, q)| {
                    [p.x, p.y, p.z, p.w].map(f64::to_bits) == [q.x, q.y, q.z, q.w].map(f64::to_bits)
                })
        })
}

/// Whether `face_name` names the report's wall: its exact name, or that
/// name with the `[k]` (decimal) suffix `ensure_unique_face_names` gives a
/// colliding name at registration. Nothing else: no prefix, no other suffix.
fn names_the_wall(face_name: &Option<String>, report_name: &Option<String>) -> bool {
    match (face_name, report_name) {
        (Some(face), Some(report)) => {
            face == report
                || face.strip_prefix(report.as_str()).and_then(|rest| rest.strip_prefix('[')).and_then(|rest| rest.strip_suffix(']')).is_some_and(
                    |index| !index.is_empty() && index.bytes().all(|byte| byte.is_ascii_digit()),
                )
        }
        (None, None) => true,
        _ => false,
    }
}

/// Consume EVERY wall-model report recorded on this thread since the last
/// call, in ONE snapshot, against ALL the bodies a call returned
/// (`(body name, solid)`): a report becomes a typed `blend.wall_model`
/// [`crate::Approximation`] on each body carrying its wall — a face with a
/// bit-identical surface AND the report's name, exactly or as registration
/// renamed it (`NAME[k]`) — and is dropped when no returned body carries it
/// (a lane that refused, a body a later step discarded). A wall carried by
/// two returned bodies (one surface, bit for bit, in both) is reported on
/// both: the reading is a property of that surface. Reports for one wall
/// collapse to the last. Never a match by name alone.
/// `take_blend_approximations(&[])` discards everything recorded (the
/// history loop's stale drain before a feature runs).
pub fn take_blend_approximations(bodies: &[(&str, &crate::BrepSolid)]) -> Vec<crate::Approximation> {
    let snapshot = BLEND_APPROXIMATIONS.with(|reports| std::mem::take(&mut *reports.borrow_mut()));
    let mut unique: Vec<WallModelReport> = Vec::new();
    for report in snapshot {
        unique.retain(|kept| !(kept.name == report.name && same_surface(&kept.surface, &report.surface)));
        unique.push(report);
    }
    let mut approximations = Vec::new();
    for (body, solid) in bodies {
        for report in &unique {
            let carried = solid.shells.iter().flat_map(|shell| &shell.faces).any(|face| {
                names_the_wall(&face.name, &report.name) && same_surface(&face.surface, &report.surface)
            });
            if carried {
                let wall = report.name.as_deref().unwrap_or("(unnamed)");
                let unaccepted = matches!(report.budget.reason, crate::BudgetReason::Unaccepted | crate::BudgetReason::Unread);
                let budget = match report.budget.mechanism {
                    crate::BudgetMechanism::LocalRounds => format!(
                        "local refinement stopped: {:?} after {} of {} rounds, {} of {} stations per segment",
                        report.budget.reason, report.budget.rounds_used, report.budget.rounds_limit,
                        report.budget.stations, report.budget.station_limit,
                    ),
                    crate::BudgetMechanism::StationRungs => format!(
                        "the open march's station ladder stopped: {:?} after {} of {} global station rungs, {} of {} stations",
                        report.budget.reason, report.budget.rounds_used, report.budget.rounds_limit,
                        report.budget.stations, report.budget.station_limit,
                    ),
                };
                let unknown = report.budget.unread > 0;
                let rails = report.budget.measured_component == crate::MeasuredComponent::Rails;
                let mut message = if !unaccepted && (rails || unknown) {
                    // A request that covers the rails and the wall says WHICH
                    // it measured, and never offers a partial reading as the
                    // worst: with unread intervals the worst is unknown.
                    let what = if rails {
                        format!("the fillet wall {wall}'s support rails stand")
                    } else {
                        format!("the fillet wall {wall} stands")
                    };
                    let against = if rails {
                        "off their exact construction (their carriers / the exact ball contacts)"
                    } else {
                        "from the rolling ball between its rails"
                    };
                    let reading = if unknown {
                        format!(
                            "an UNKNOWN distance {against}: {} interval(s) could not be read (the largest READABLE reading is {:.3e} mm, not the worst)",
                            report.budget.unread, report.residual
                        )
                    } else {
                        format!("{:.3e} mm {against}", report.residual)
                    };
                    format!(
                        "{what} {reading}, against a construction request of {:.3e} mm (accepted at the intersection-fit bar); {budget}",
                        report.request,
                    )
                } else if unaccepted {
                    format!(
                        "the blend wall {wall} ships from a rung that never met acceptance: its worst reading \
                         {:.3e} mm against its acceptance bar {:.3e} mm; {budget}",
                        report.residual, report.request,
                    )
                } else {
                    format!(
                        "the fillet wall {wall} stands {:.3e} mm from the rolling ball between its rails, against a \
                         construction request of {:.3e} mm (accepted at the intersection-fit bar); {budget}",
                        report.residual, report.request,
                    )
                };
                if let Some(detail) = &report.detail {
                    message = format!("{message} ({detail})");
                }
                approximations.push(crate::Approximation {
                    code: if unaccepted { "blend.wall_acceptance" } else { "blend.wall_model" }.to_string(),
                    body: body.to_string(),
                    measured: report.residual,
                    bar: report.request,
                    volume_bound: None,
                    edges: Vec::new(),
                    budget: Some(report.budget.clone()),
                    message,
                });
            }
        }
    }
    approximations
}


pub use chain::{blend_smooth_chain, blend_smooth_chain_if_closed};
pub(crate) use corner::round_concave_chain_corner;
pub use corner::round_convex_corner;
pub use edge::{blend_closed_edge, blend_edge_variable, blend_open_edge};
pub(crate) use collapse::is_full_width_collapse;
pub(crate) use planar_chart::{
    fit_planar_charts_to_trims, is_planar_chart_refusal, planar_chart_overrun,
};
pub(crate) use track_fit::PCURVE_OFF_FLOOR;
pub(crate) use edge::{consumed_band, is_consumed_snap, is_rail_collapse};
pub(crate) use network::{
    blend_star_network, blend_star_network_with_corner_policy, degenerate_corner_setbacks, mixed_convexity_corner, mixed_convexity_corners, selection_convexity, DegenerateSetback,
    MixedCorner,
};
pub(crate) use miter::{is_marched_fit_off_carriers, is_miter_support_overrun};
pub(crate) use runout::{build_runout_walls, plan_runout, take_runout_reports, RunoutPlan, RunoutWallReport};
pub(crate) use fold::{is_wall_fold, WALL_FOLDS};
pub(crate) use stations::is_ball_off_carrier;
