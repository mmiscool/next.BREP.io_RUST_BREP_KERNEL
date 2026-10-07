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
    KernelRefusal, NurbsCurve, NurbsSurface, Vec3, Vec4,
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

/// Why one body did not import: the builders' legacy text, or a TYPED refusal
/// from the one gate that mints one today, the shell-closure gate
/// (`builder/closure.rs`). The class rides to the feature boundary through
/// [`import_step_bodies`]; every other producer inside the importer still
/// returns text and is preserved verbatim (typed-refusal plan, item 1).
#[derive(Clone, Debug, PartialEq)]
pub enum StepBodyError {
    Text(String),
    Refused(crate::KernelRefusal),
}

impl StepBodyError {
    /// The human text either way — the refusal's own message, verbatim.
    pub fn message(&self) -> &str {
        match self {
            Self::Text(text) => text,
            Self::Refused(refusal) => &refusal.message,
        }
    }

    /// Whether this is the shell-closure gate's refusal
    /// (`RefusalClass::UnsoundResult { defect: SoundnessDefect::VectorArea }`,
    /// `builder/closure.rs`): the one typed class the body lanes treat as a
    /// REFUSED body rather than a body that failed to build. Every other
    /// producer that went typed on 2026-10-03 (the pinched-vertex split, the
    /// `BREP_WITH_VOIDS` certifications, the post-build validation) keeps the
    /// outcome its text had: counted in `failed`, skipped on an opportunistic
    /// surface-model sheet, never a whole-file refusal.
    pub fn closure_refusal(&self) -> bool {
        matches!(
            self,
            Self::Refused(crate::KernelRefusal {
                class: crate::RefusalClass::UnsoundResult {
                    defect: crate::SoundnessDefect::VectorArea,
                    ..
                },
                ..
            })
        )
    }

    /// The same error with its text rewritten by `f`, the class untouched.
    pub fn with_context(self, f: impl FnOnce(&str) -> String) -> Self {
        match self {
            Self::Text(text) => Self::Text(f(&text)),
            Self::Refused(refusal) => Self::Refused(refusal.with_message(f)),
        }
    }
}

impl From<String> for StepBodyError {
    fn from(text: String) -> Self {
        Self::Text(text)
    }
}

impl From<&str> for StepBodyError {
    fn from(text: &str) -> Self {
        Self::Text(text.to_string())
    }
}

impl From<crate::KernelRefusal> for StepBodyError {
    fn from(refusal: crate::KernelRefusal) -> Self {
        Self::Refused(refusal)
    }
}

impl std::fmt::Display for StepBodyError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.message())
    }
}

/// Public entry: parse a STEP Part 21 document and return one `BrepSolid` per
/// MANIFOLD_SOLID_BREP, FACETED_BREP, or certified BREP_WITH_VOIDS. Every
/// returned solid passes `validate()` and closes its shells' vector area to
/// the import's bar; a file with a body the closure gate refuses is refused
/// as a whole (see [`import_step_bodies`]).
pub fn import_step(text: &str) -> Result<Vec<BrepSolid>, String> {
    Ok(import_step_with_appearance(text)?.0)
}

/// [`import_step_with_appearance`] with the error TYPED: the lane the IMPORT3D
/// feature uses, so a refused body reaches the app as a class and not only as
/// text.
///
/// **A refused body refuses the import.** `collect_step_solids` skips a body
/// that failed to build and returns the rest, counting it in `failed`, and
/// until 2026-09-30 this entry point returned those bodies as a success —
/// the count and the first error never left `ImportedBodies`, so a file with
/// a body the importer could not build imported SILENTLY short. A body the
/// closure gate refuses is a body the file describes and the kernel declines
/// to certify, and returning the others as the file's contents would be the
/// partial result the typed-refusal plan rules out ("partial geometry is not a
/// successful result"). The feature has no per-body typed slot to say "five
/// of seven" (`Fulfilment::rejected` is a closed selection-resolution kind),
/// so the whole import is refused with the first body's class and a message
/// naming every refused body; the report lane ([`import_step_report`]) keeps
/// returning the bodies that did build beside the refusals, for tools that
/// want to read them. Bodies that fail with TEXT keep the pre-2026-09-30
/// behaviour (skipped, the rest returned) — changing that is the plan's
/// migration of the builder boundary, not this gate's.
pub fn import_step_bodies(text: &str) -> Result<ImportedStep, StepBodyError> {
    let imported = collect_step_solids(text)?;
    if let Some(first) = imported.refused_bodies.first() {
        let others = imported.refused_bodies.len() - 1;
        let refusal = if others == 0 {
            first.clone()
        } else {
            let rest: Vec<String> = imported.refused_bodies[1..]
                .iter()
                .map(|refusal| refusal.message.clone())
                .collect();
            first.clone().with_message(|message| {
                format!("{message} And {others} more: {}", rest.join(" "))
            })
        };
        return Err(StepBodyError::Refused(refusal.with_message(|message| {
            format!(
                "{} of {} bodies refused as open; {}",
                imported.refused_bodies.len(),
                imported.solids.len() + imported.failed,
                message
            )
        })));
    }
    if imported.solids.is_empty() {
        return Err(StepBodyError::Text(imported.first_error.unwrap_or_else(|| {
            "step_import: no MANIFOLD_SOLID_BREP/FACETED_BREP/BREP_WITH_VOIDS body imported (unsupported or invalid representation)"
                .into()
        })));
    }
    let approximations = imported
        .closure_notes
        .iter()
        .enumerate()
        .filter_map(|(index, note)| note.approximation().map(|a| (index, a.clone())))
        .collect();
    Ok(ImportedStep {
        solids: imported.solids,
        appearances: imported.appearances,
        approximations,
    })
}

/// What [`import_step_bodies`] returns: the bodies, their colours, and the
/// typed approximations the closure gate attached to some of them.
#[derive(Clone, Debug)]
pub struct ImportedStep {
    /// One solid per body that reconstructed, in file order.
    pub solids: Vec<BrepSolid>,
    /// One per solid, in the same order (default when the file has none).
    pub appearances: Vec<BodyAppearance>,
    /// `(index into solids, approximation)` for every body the closure gate
    /// imported over its bar (`builder/closure.rs`): a typed statement that
    /// the body's volume is determined only to the bound the approximation
    /// carries. Empty on a file whose every body closes.
    pub approximations: Vec<(usize, crate::Approximation)>,
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
    import_step_bodies(text)
        .map(|imported| (imported.solids, imported.appearances))
        .map_err(|error| error.to_string())
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
    /// The bodies the shell-closure gate refused, typed
    /// (`RefusalClass::UnsoundResult { defect: SoundnessDefect::VectorArea }`),
    /// in file order. Each is counted in `failed`. [`import_step`] and the
    /// feature lane refuse the whole file on any of these; this lane returns
    /// the bodies that did import beside them.
    pub refused: Vec<crate::KernelRefusal>,
    /// The bodies that FAILED to build with a typed cause that is not the
    /// closure gate's, in file order: the pinched-vertex split, a
    /// `BREP_WITH_VOIDS` certification, the post-build validation
    /// (2026-10-03). Each is counted in `failed`, and the first is
    /// `first_error`'s text; none refuses the whole file on any lane, which
    /// is the outcome their text had.
    pub failed_refusals: Vec<crate::KernelRefusal>,
    /// `(index into solids, approximation)` for every imported body whose
    /// shell closes over its bar (`import.shell_closure`); see
    /// [`ImportedStep::approximations`].
    pub approximations: Vec<(usize, crate::Approximation)>,
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
/// visible (2026-09-26). Since 2026-10-03 both numbers are read BETWEEN the
/// fit's stations too (`geometry::trim_floor_reading`) and the class is the
/// corrected bar's: a trim within the floor of the file curve's standoff is
/// the file's residual whatever its image does along the carrier.
pub const TRIM_BOUNDED: &str = "import.trim_bounded";
pub const TRIMS_BOUNDED: &str = "import.trims_bounded";
pub const TRIMS_FITTER_MISS: &str = "import.trims_fitter_miss";
pub const TRIM_BOUND_MM: &str = "import.trim_bound_mm";
/// Bounded trims whose fit was CLAMPED into the carrier's chart, and the worst
/// 3D distance that clamp corresponds to (2026-09-30; a report, not a bar).
pub const TRIMS_CLAMPED: &str = "import.trims_clamped";
pub const TRIM_CLAMP_MM: &str = "import.trim_clamp_mm";

/// The shell-closure gate's readings on the bodies it ACCEPTED
/// (`builder/closure.rs`): the worst residual-to-bar ratio over the accepted
/// bodies, how many bodies the scan could not judge (span budget or an
/// unreadable face — accepted unjudged, one Degraded event each), and how
/// many bodies it refused (also in [`StepImportReport::refused`]).
pub const CLOSURE_RATIO_MAX: &str = "import.closure_ratio_max";
pub const CLOSURE_APPROXIMATE: &str = "import.closure_approximate";
pub const CLOSURE_UNJUDGED: &str = "import.closure_unjudged";
pub const CLOSURE_REFUSED: &str = "import.closure_refused";

fn closure_diagnostics(
    notes: &[builder::ClosureNote],
    refused: &[crate::KernelRefusal],
    diagnostics: &mut crate::KernelDiagnostics,
) {
    for (index, note) in notes.iter().enumerate() {
        match note {
            builder::ClosureNote::Closed { residual, bar } => {
                diagnostics.measure_max(CLOSURE_RATIO_MAX, residual / bar);
            }
            builder::ClosureNote::Approximate(approximation) => {
                diagnostics.measure_max(CLOSURE_RATIO_MAX, approximation.measured / approximation.bar);
                diagnostics.count(CLOSURE_APPROXIMATE);
                diagnostics.event(
                    crate::DiagnosticSeverity::Degraded,
                    crate::KernelStage::Validate,
                    CLOSURE_APPROXIMATE,
                    format!("body {index}: {}", approximation.message),
                );
            }
            builder::ClosureNote::Unmeasured { reason, spans } => {
                diagnostics.count(CLOSURE_UNJUDGED);
                diagnostics.event(
                    crate::DiagnosticSeverity::Degraded,
                    crate::KernelStage::Validate,
                    CLOSURE_UNJUDGED,
                    format!("body {index}: shell closure not judged — {reason} ({spans} spans); the body is accepted unjudged, not certified closed."),
                );
            }
            builder::ClosureNote::Unreadable { first, count } => {
                diagnostics.count(CLOSURE_UNJUDGED);
                diagnostics.event(
                    crate::DiagnosticSeverity::Degraded,
                    crate::KernelStage::Validate,
                    CLOSURE_UNJUDGED,
                    format!("body {index}: shell closure not judged — {count} face(s) unreadable by the scan, first: {first}; the body is accepted unjudged, not certified closed."),
                );
            }
            builder::ClosureNote::Unjudged => {
                diagnostics.count(CLOSURE_UNJUDGED);
            }
            builder::ClosureNote::Overridden(refusal) => {
                diagnostics.count(CLOSURE_UNJUDGED);
                diagnostics.event(
                    crate::DiagnosticSeverity::Degraded,
                    crate::KernelStage::Validate,
                    CLOSURE_UNJUDGED,
                    format!("body {index}: the closure guard is OFF (BREP_IMPORT_CLOSURE_GUARD=0); it would have refused: {}", refusal.message),
                );
            }
        }
    }
    diagnostics.count_n(CLOSURE_REFUSED, refused.len() as u64);
}

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
        let clamped = trims.iter().filter(|trim| trim.clamped_excursion > 0.0).count();
        let worst_clamp = trims.iter().map(|trim| trim.clamped_distance).fold(0.0f64, f64::max);
        diagnostics.count_n(TRIMS_BOUNDED, trims.len() as u64);
        diagnostics.count_n(TRIMS_FITTER_MISS, fitters as u64);
        diagnostics.measure_max(TRIM_BOUND_MM, worst.residual);
        diagnostics.count_n(TRIMS_CLAMPED, clamped as u64);
        diagnostics.measure_max(TRIM_CLAMP_MM, worst_clamp);
        diagnostics.event(
            crate::DiagnosticSeverity::Degraded,
            crate::KernelStage::Collect,
            TRIM_BOUNDED,
            format!(
                "body {index}: {} trim(s) sit off the {:.0e} mm fit bar and carry their measured residual as their band ({} of them the file's residual, within the floor of the curve's standoff read between the stations; {} the fitter's miss); worst ADVANCED_FACE #{} (surface #{}) edge {}: {:.3e} mm, its curve {:.3e} mm off the carrier, the image {:.3e} mm off its feet ({}; {} at {} samples).{}",
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
                worst.samples,
                if clamped > 0 {
                    format!(" {clamped} of them were clamped into their chart, by up to {worst_clamp:.3e} mm.")
                } else {
                    String::new()
                }
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
    closure_diagnostics(&imported.closure_notes, &imported.refused_bodies, &mut diagnostics);
    Ok(StepImportReport {
        solids: imported.solids,
        failed: imported.failed,
        first_error: imported.first_error,
        refused: imported.refused_bodies,
        failed_refusals: imported.failed_refusals,
        approximations: imported
            .closure_notes
            .iter()
            .enumerate()
            .filter_map(|(index, note)| note.approximation().map(|a| (index, a.clone())))
            .collect(),
        stated_precisions_mm: imported.stated_precisions_mm,
        diagnostics,
    })
}
