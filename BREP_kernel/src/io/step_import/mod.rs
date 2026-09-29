//! STEP Part 21 import for NURBS and analytic surfaces, assemblies, and PMI.
//!
//! The importer reconstructs and validates BREP topology. Analytic carriers are
//! converted to NURBS sized to the face bounds. Seam edges occupy opposite
//! parameter boundaries, and collapsed pole bounds become degenerate edges so
//! faces close and integrate correctly.
//!
//! Pcurves are DERIVED, by projecting each edge onto its carrier; a
//! `SURFACE_CURVE`/`SEAM_CURVE` bundle contributes only its 3D curve.
//!
//! [`supplied`] can read the bundle's `PCURVE` associations instead and seat
//! the vendor's own stated trim, and does so correctly, but it is OFF unless
//! `BREP_SUPPLIED_PCURVES=1`. Not caution — measurement. Preferring a supplied
//! pcurve helps only where the edge's 3D curve has drifted off its own carrier,
//! and nothing distinguishes that case from the ones where it hurts: our own
//! exporter writes exact analytic 3D curves with fitted pcurves, so seating the
//! fit cost a round trip through our writer a factor of 4800 in volume
//! fidelity. See `supplied::supplied_pcurves_enabled` for the full derivation.

use crate::topology::{
    BrepSolid, CoedgeRecord, EdgeRecord, FaceRecord, LoopRecord, ShellRecord, VertexRecord,
};
use crate::{
    build_pcurve_on_surface_range, make_arc, make_extrusion, make_hyperbola, make_line,
    make_parabola, make_plane, make_revolution, offset_surface, transform_brep, AffineTransform,
    NurbsCurve, NurbsSurface, Vec3, Vec4,
};
use rustc_hash::{FxHashMap as HashMap, FxHashSet as HashSet};
use std::collections::VecDeque;

const TAU: f64 = std::f64::consts::TAU;

pub(crate) mod parse;
mod supplied;
mod bodies;
mod assembly;
mod assembly_pmi;
mod mapped;
mod builder;
mod geometry;
mod styles;
mod pmi;
mod precision;
mod trim_readings;

use crate::appearance::{BodyAppearance, ImportedColor};
use parse::*;
use supplied::*;
use bodies::*;
use mapped::*;
use builder::*;
use geometry::*;
use styles::*;
pub use assembly::{read_step_assembly, StepAssembly, StepOccurrence, StepProduct};
pub use assembly_pmi::{occurrence_ref_parts, rewrite_occurrence_refs, OCCURRENCE_REF_PREFIX};
pub use mapped::StepMappedItemError;
pub use pmi::read_step_pmi;
pub use precision::STATED_PRECISION_INCONSISTENCY_RATIO;
pub use trim_readings::{
    import_step_trim_readings, StepBodyTrimReadings, StepEdgeReading, StepFaceReading,
    StepSuppliedTrim, StepTrimReadings,
};
pub(crate) use pmi::decode_step_text as decode_step_text_for_tests;


// ---------------------------------------------------------------------------
// Topology assembly
// ---------------------------------------------------------------------------

/// Public entry: parse a STEP Part 21 document and return one `BrepSolid` per
/// MANIFOLD_SOLID_BREP, FACETED_BREP, or certified BREP_WITH_VOIDS. Every
/// returned solid passes `validate()`.
pub fn import_step(text: &str) -> Result<Vec<BrepSolid>, String> {
    Ok(import_step_with_appearance(text)?.0)
}

/// [`import_step`] plus the file's PRESENTATION colours: one
/// [`BodyAppearance`] per returned solid, in the same order.
///
/// This is the lane the IMPORT3D feature uses, because colour is stamped as
/// scene metadata on the names IT chooses — see
/// `feature_pipeline::features::import3d` and `io/appearance.rs` for the record
/// shape. A file with no presentation entities returns default (empty)
/// appearances, never a short vector, so callers can `zip` unconditionally.
pub fn import_step_with_appearance(
    text: &str,
) -> Result<(Vec<BrepSolid>, Vec<BodyAppearance>), String> {
    let imported = collect_step_solids(text)?;
    if imported.solids.is_empty() {
        return Err(imported.first_error.unwrap_or_else(|| {
            "step_import: no MANIFOLD_SOLID_BREP/FACETED_BREP/BREP_WITH_VOIDS body imported (unsupported or invalid representation)"
                .into()
        }));
    }
    Ok((imported.solids, imported.appearances))
}

/// What [`import_step_report`] returns: the bodies, how gracefully the file
/// imported, and the import's diagnostics.
#[derive(Clone, Debug)]
pub struct StepImportReport {
    /// One solid per body that reconstructed, in file order.
    pub solids: Vec<BrepSolid>,
    /// Things that did not come in (skipped, not fatal): a body that failed to
    /// reconstruct, an occurrence whose transform failed, or a mapping that was
    /// refused (`mapped::StepMappedItemError`).
    pub failed: usize,
    /// The first such failure's text, when `failed > 0`.
    pub first_error: Option<String>,
    /// Every precision the file states, in millimetres, ascending — see
    /// `parse::stated_precisions_mm`. Empty when it states none.
    pub stated_precisions_mm: Vec<f64>,
    /// The import's diagnostics: measurements always, plus one
    /// `import.stated_precision_inconsistent` warning when the file's stated
    /// precision is far under the deviation its own geometry shows
    /// ([`precision::stated_precision_diagnostics`]). Informational only —
    /// acceptance never depends on it.
    pub diagnostics: crate::KernelDiagnostics,
}

/// The event code for a body that carries a derived trim off the fit bar (the
/// 1e-7 floor plus its stations' standoff) after the refit at the floor, which
/// the importer accepts as it did every fit before the bar, and the counters
/// and measure beside it. The residual
/// is the band `EntityTolerances` measures for that edge; the event names how
/// many are the file's residual and how many the fitter's miss, and the worst
/// with its numbers, so what the importer accepted before the bar silently is
/// visible (2026-09-26).
pub const TRIM_BOUNDED: &str = "import.trim_bounded";
pub const TRIMS_BOUNDED: &str = "import.trims_bounded";
pub const TRIMS_FITTER_MISS: &str = "import.trims_fitter_miss";
pub const TRIM_BOUND_MM: &str = "import.trim_bound_mm";

fn bounded_trim_diagnostics(
    bounded: &[Vec<builder::readings::BoundedTrim>],
    diagnostics: &mut crate::KernelDiagnostics,
) {
    for (index, trims) in bounded.iter().enumerate() {
        let Some(worst) = trims
            .iter()
            .max_by(|a, b| a.residual.partial_cmp(&b.residual).unwrap_or(std::cmp::Ordering::Equal))
        else {
            continue;
        };
        let fitters = trims.iter().filter(|trim| trim.fitter).count();
        diagnostics.count_n(TRIMS_BOUNDED, trims.len() as u64);
        diagnostics.count_n(TRIMS_FITTER_MISS, fitters as u64);
        diagnostics.measure_max(TRIM_BOUND_MM, worst.residual);
        diagnostics.event(
            crate::DiagnosticSeverity::Degraded,
            crate::KernelStage::Collect,
            TRIM_BOUNDED,
            format!(
                "body {index}: {} trim(s) sit off the {:.0e} mm fit bar and carry their measured residual as their band ({} of them the file's residual, the image on the stations' feet; {} the fitter's miss); worst ADVANCED_FACE #{} (surface #{}) edge {}: {:.3e} mm, its curve {:.3e} mm off the carrier, the image {:.3e} mm off its feet ({}; {} at {} samples).",
                trims.len(),
                crate::PCURVE_REFINEMENT_TOLERANCE,
                trims.len() - fitters,
                fitters,
                worst.face_ref,
                worst.surface_ref,
                worst.edge_id,
                worst.residual,
                worst.standoff,
                worst.image_to_foot,
                if worst.fitter { "the fitter's miss" } else { "the file's residual" },
                worst.exit,
                worst.samples
            ),
        );
    }
}

/// Like [`import_step`] but reports how many bodies failed and the first error,
/// so callers can distinguish a fully- from a partially-imported assembly —
/// and runs the stated-precision consistency check over the imported bodies,
/// whose result rides in [`StepImportReport::diagnostics`].
///
/// The check measures every edge and vertex of every body (the same
/// `EntityTolerances` reading the per-entity-tolerance census took), which is
/// why it lives on this lane and not on [`import_step_with_appearance`]: the
/// feature pipeline has no diagnostics channel to carry the answer, and a
/// measurement nobody reads is not worth its cost on every IMPORT3D.
pub fn import_step_report(text: &str) -> Result<StepImportReport, String> {
    let imported = collect_step_solids(text)?;
    let mut diagnostics =
        precision::stated_precision_diagnostics(&imported.stated_precisions_mm, &imported.solids);
    bounded_trim_diagnostics(&imported.bounded_trims, &mut diagnostics);
    Ok(StepImportReport {
        solids: imported.solids,
        failed: imported.failed,
        first_error: imported.first_error,
        stated_precisions_mm: imported.stated_precisions_mm,
        diagnostics,
    })
}
