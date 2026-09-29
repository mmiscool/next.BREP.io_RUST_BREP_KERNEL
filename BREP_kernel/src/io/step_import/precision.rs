//! Stated precision as a consistency check at import.
//!
//! A STEP file states its own precision in `UNCERTAINTY_MEASURE_WITH_UNIT`.
//! Seeding a per-entity tolerance band from that figure was measured across
//! the 54-fixture corpus and DROPPED: the figure is not a property of the
//! geometry — half the corpus states 1e-17 (exporter boilerplate seven
//! orders under the policy floor), three files state several values at once,
//! and where a seed would act at all it could only widen a band on the
//! exporter's word. What survives is the other direction of the same
//! comparison: a file whose stated figure is far UNDER the deviation its own
//! edges show is misdescribing itself, and the import should say so.
//!
//! This module is that diagnostic. It compares the loosest figure the file
//! states against the largest edge-curve-vs-carrier disagreement and vertex
//! gap the imported bodies measure (`EntityTolerances`, the same reading the
//! census took, unclamped), records both as measurements, and emits one
//! warning when the measured deviation exceeds the stated figure by more
//! than [`STATED_PRECISION_INCONSISTENCY_RATIO`]. It never changes
//! acceptance: nothing here is a band, a gate, or an input to one.
//!
//! Units are load-bearing. The stated figure arrives already scaled to
//! millimetres through the unit its own entity names
//! (`parse::stated_precisions_mm`); 31 of the 48 corpus fixtures that state a
//! precision declare metres, and comparing the raw figure inverts the answer.

use super::BrepSolid;
use crate::{DiagnosticSeverity, EntityTolerances, KernelDiagnostics, KernelStage};

/// How far the measured deviation may exceed the stated precision before the
/// file is called inconsistent with itself.
///
/// A decade, set from the corpus (54 files). Of the 48 that state a
/// precision, 25 state exporter boilerplate (`1.E-17` m = 1e-14 mm, under
/// double resolution at any part size) and measure 3e4× to 5e14× over it; 14
/// state a real export tolerance their geometry meets, at 0.07× to 2.3× (two
/// outliers at 4.8× and 9.0×); and 9 state a real figure their geometry
/// misses by 12× to 5e4× (`EngineBlock` 1e-7 mm against 5.1e-3 measured;
/// `raspberry_pi_3_molex` 1e-5 against 9.8e-4). A decade sits in the gap
/// between the second and third groups, says "an order of magnitude" in the
/// same breath it names the numbers, and leaves headroom for what the
/// measurement itself cannot see below: the edge reading is the vendor's 3D
/// curve against the kernel's OWN projected pcurve, whose fit residual is of
/// order 1e-6..1e-5 mm, so a stated 5e-6 mm cannot be judged finer than that.
pub const STATED_PRECISION_INCONSISTENCY_RATIO: f64 = 10.0;

/// Measurement keys the report always carries when the file states a
/// precision, so a caller can read the comparison whether or not it fired.
pub const STATED_PRECISION_MM: &str = "import.stated_precision_mm";
pub const STATED_PRECISION_TIGHTEST_MM: &str = "import.stated_precision_tightest_mm";
pub const MEASURED_EDGE_DEVIATION_MM: &str = "import.measured_edge_deviation_mm";
pub const MEASURED_VERTEX_GAP_MM: &str = "import.measured_vertex_gap_mm";
/// The event code, and the counter of how many bodies were measured.
pub const STATED_PRECISION_INCONSISTENT: &str = "import.stated_precision_inconsistent";
pub const MEASURED_BODIES: &str = "import.stated_precision.measured_bodies";

/// The stated-precision consistency check over `solids`, which imported from
/// a file stating `stated_mm` (ascending, in mm; empty when it states none).
///
/// A file that states nothing is not measured — there is nothing to compare
/// against, and the measurement is the cost of walking every edge. A file
/// that states several figures is judged against the LOOSEST: the check is
/// meant to be believed when it fires, and a file with one claim that covers
/// its deviation is not unambiguously misdescribing itself. The tightest is
/// recorded beside it so the spread is visible.
///
/// Every measurement is taken through `EntityTolerances::measured_*`, the
/// unclamped readings, so a defective entity reports its real deviation and
/// not the cap. A measurement that cannot be taken is skipped; it can only
/// make the check quieter, never louder, and never fails the import.
pub(super) fn stated_precision_diagnostics(
    stated_mm: &[f64],
    solids: &[BrepSolid],
) -> KernelDiagnostics {
    let mut diagnostics = KernelDiagnostics::default();
    let (Some(&tightest), Some(&loosest)) = (stated_mm.first(), stated_mm.last()) else {
        return diagnostics;
    };
    diagnostics.measure_max(STATED_PRECISION_MM, loosest);
    diagnostics.measure_max(STATED_PRECISION_TIGHTEST_MM, tightest);

    // (deviation, solid index, entity id) — the worst edge and vertex.
    let mut worst_edge: Option<(f64, usize, u64)> = None;
    let mut worst_vertex: Option<(f64, usize, u64)> = None;
    let mut scale = 0.0f64;
    for (index, solid) in solids.iter().enumerate() {
        diagnostics.count(MEASURED_BODIES);
        scale = scale.max(crate::solid_scale(solid));
        let mut tolerances = EntityTolerances::with_floor(solid, 0.0);
        for edge in &solid.edges {
            if edge.degenerate {
                continue;
            }
            if let Some(deviation) = tolerances.measured_edge_deviation(edge.id) {
                if worst_edge.is_none_or(|(w, _, _)| deviation > w) {
                    worst_edge = Some((deviation, index, edge.id));
                }
            }
        }
        for vertex in &solid.vertices {
            if let Some(gap) = tolerances.measured_vertex_gap(vertex.id) {
                if worst_vertex.is_none_or(|(w, _, _)| gap > w) {
                    worst_vertex = Some((gap, index, vertex.id));
                }
            }
        }
    }
    if let Some((deviation, _, _)) = worst_edge {
        diagnostics.measure_max(MEASURED_EDGE_DEVIATION_MM, deviation);
    }
    if let Some((gap, _, _)) = worst_vertex {
        diagnostics.measure_max(MEASURED_VERTEX_GAP_MM, gap);
    }

    // The worse of the two readings decides; the message names whichever it is.
    let worst = match (worst_edge, worst_vertex) {
        (Some(edge), Some(vertex)) => {
            if edge.0 >= vertex.0 {
                Some(("edge", edge))
            } else {
                Some(("vertex", vertex))
            }
        }
        (Some(edge), None) => Some(("edge", edge)),
        (None, Some(vertex)) => Some(("vertex", vertex)),
        (None, None) => None,
    };
    let Some((kind, (measured, index, id))) = worst else {
        return diagnostics;
    };
    if !(measured > loosest * STATED_PRECISION_INCONSISTENCY_RATIO) {
        return diagnostics;
    }
    let ratio = measured / loosest;
    let declarations = if stated_mm.len() > 1 {
        format!(
            " (the loosest of {} distinct figures, tightest {:.3e} mm)",
            stated_mm.len(),
            tightest
        )
    } else {
        String::new()
    };
    // A figure under the resolution of a double at the part's own size is not
    // a claim any geometry could meet, whatever the exporter wrote.
    let resolution = f64::EPSILON * scale;
    let boilerplate = if loosest < resolution {
        format!(
            " The stated figure is below double resolution at this part's size ({:.1e} mm), so it cannot describe any geometry.",
            resolution
        )
    } else {
        String::new()
    };
    diagnostics.event(
        DiagnosticSeverity::Warning,
        KernelStage::Collect,
        STATED_PRECISION_INCONSISTENT,
        format!(
            "the file states a precision of {loosest:.3e} mm{declarations}, but its own geometry disagrees with itself by up to {measured:.3e} mm ({kind} {id} of body {index}), {ratio:.3e}× the stated figure: the stated precision does not describe this geometry.{boilerplate} Acceptance is unchanged."
        ),
    );
    diagnostics
}
