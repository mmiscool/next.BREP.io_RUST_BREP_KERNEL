pub mod profile;
pub use crate::arrangement::Vec2;
use crate::classification::{classify_point, parameter_point_in_face, PolygonClass};
pub use crate::fragment::{FaceFragmentRecord, FragmentEdgeSource};
pub use crate::imprint::{
    EdgeSplitRecord, FaceImprints, FaceKey, ImprintOptions, ImprintPieceRecord,
    ImprintResultRecord, ImprintVertex,
};
use crate::tolerance::{assembler_weld, commit_weld};
pub use crate::topology::{
    adaptive_coedge_error, BrepSolid, CoedgeRecord, EdgeRecord, FaceRecord, LoopRecord,
    ShellRecord, VertexRecord,
};
pub use crate::{
    KernelRefusal, OrRefuse, RefusalClass,
    apply_edge_splits, apply_edge_splits_with_map, build_imprints, build_pcurve_on_surface,
    build_pcurve_on_surface_range, classify_surface_pair,
    fragment_solid, interpolate_curve, merge_curve_continuation_edges,
    merge_same_surface_faces_excluding, merge_same_surface_faces_scoped,
    project_point_to_curve, project_point_to_surface, solid_signed_volume, solid_signed_volume_for_sign, AffineTransform,
    DiagnosticSeverity, KernelDiagnostics, KernelOutcome, KernelStage, KernelTolerances, NurbsCurve,
    PointClass, SolidClassifier, SurfacePairRelation, Vec3,
};
pub use rustc_hash::{FxHashMap as HashMap, FxHashSet as HashSet};
use serde::{Deserialize, Serialize};
pub use web_time::Instant;

thread_local! {
    /// Running count (per thread) of one-use edges the EDGE-CONFORMANCE
    /// repair lanes merged or bridged.  The lanes only fire on an assembly
    /// that would otherwise refuse, so a zero delta across an operation is
    /// an honest witness that the boolean assembled cleanly without the
    /// repair.  Test-observable; not part of any result.
    pub static CONFORMANCE_REPAIRS: std::cell::Cell<u64> =
        const { std::cell::Cell::new(0) };
}

#[derive(Clone, Copy, Debug, Deserialize, Serialize)]
#[serde(rename_all = "lowercase")]
pub enum BooleanOperation {
    Union,
    Intersect,
    Subtract,
}

#[derive(Clone, Debug, Deserialize)]
pub struct BooleanOptions {
    #[serde(default = "default_tolerance")]
    pub tolerance: f64,
    /// Optional complete accuracy/search policy.  `tolerance` remains the
    /// backwards-compatible model-identity shorthand when this is absent.
    #[serde(default)]
    pub tolerances: Option<KernelTolerances>,
    #[serde(default)]
    pub imprint: ImprintOptions,
    /// Coalesce adjacent result fragments after assembly. This defaults to
    /// true to preserve the kernel's normal clean-boundary behavior.
    #[serde(default = "default_true")]
    pub merge_coplanar_faces: bool,
    /// Face-name substrings that PIN a result face out of the coplanar/cosurface
    /// merge (only consulted when [`Self::merge_coplanar_faces`] is on). A face whose
    /// `name` CONTAINS any of these is emitted unchanged instead of being
    /// coalesced with a mergeable neighbour; faces matching none merge exactly as
    /// before. Default empty — no caller sees a behavior change unless it opts
    /// in. Sheet metal uses this to keep every thickness/side wall face per
    /// outline segment in the folded solid (so a flange can still attach to a
    /// specific segment: fusing two collinear thickness faces would erase the
    /// segment boundary).
    #[serde(default)]
    pub keep_unmerged_name_substrs: Vec<String>,
    /// On by default since the 2026-10-10 census read the 94-case manifest
    /// unchanged with it; `BREP_UNTOUCHED_PASSTHROUGH` overrides either way
    /// (`0` off, anything else on).
    #[serde(default = "default_true")]
    pub untouched_passthrough: bool,
    /// On by default since the 2026-10-10 census;
    /// `BREP_DISJOINT_FAST_PATH` overrides either way (`0` off, anything
    /// else on).
    #[serde(default = "default_true")]
    pub disjoint_fast_path: bool,
    /// Lane X row 12: when, in at least two axes, one operand's control-net
    /// box ends at or before the other's begins (exactly, no band), the
    /// operands can share no interior and touch on a line or at a point: the
    /// disjoint path's answers hold. Every control weight must be positive
    /// (the hull property), both operands plain solids, and the caller must
    /// say what follows (`seam_reseat_later_references` is `Some`), so a
    /// caller that cannot keeps today's ids. On by default since the row 12
    /// census; `BREP_CONTACT_FAST_PATH` overrides either way (`0` off,
    /// anything else on).
    #[serde(default = "default_true")]
    pub contact_fast_path: bool,
    /// OFF by default: the 94-case census read unchanged with it, but the
    /// kernel battery's `offset_subtract_fixture` pins a result whose
    /// untouched planar face carries an open trim joint of 8.7e-4 from its
    /// coarse polyline operand, which the whole-solid joints pass closes and
    /// the scoped pass leaves as the operand had it; and the row buys no
    /// measurable time (the record of 2026-10-10). `BREP_TOUCHED_ASSEMBLY`
    /// overrides either way (`0` off, anything else on).
    #[serde(default)]
    pub touched_assembly: bool,
    /// On by default since the 2026-10-11 census (the whole kernel battery,
    /// the 94-case manifest and `hlr_tests` green with it on, every manifest
    /// fragment the sector decides agreeing with the ray cast);
    /// `BREP_SECTOR_CLASS` overrides either way (`0`/`off` the escape hatch,
    /// `check` for the differential run that logs disagreements to
    /// `BREP_SECTOR_CENSUS`, anything else on). Appended at the end of the
    /// struct (2026-10-11).
    #[serde(default = "default_true")]
    pub sector_classification: bool,
    /// Non-manifold plan 1.6: trim a declared Sheet by a Solid with Intersect.
    /// On after the 2026-10-14 Sheet census and whole-kernel qualification.
    /// BREP_SHEET_TRIM overrides this option (1 enables, 0 disables).
    #[serde(default = "default_true")]
    pub sheet_trim: bool,
    /// Preserve the closed operand regions at a lower-dimensional union
    /// contact as explicit cells. Opt-in pending the Cellular census.
    /// BREP_KEEP_CELLS overrides this option (1 enables, 0 disables).
    #[serde(default)]
    pub keep_cells: bool,
    /// The references later features hold (their `inputParams` text), when
    /// the caller knows them: `Some` admits the operand seam re-seat
    /// (`BREP_SEAM_RESEAT`, lane X row 11b) for every cylinder wall none of
    /// them names; `None` (the caller cannot tell) never re-seats.
    #[serde(default)]
    pub seam_reseat_later_references: Option<Vec<String>>,
    /// The references later features hold, as for
    /// [`Self::seam_reseat_later_references`]: `Some` admits the seam rejoin
    /// (`BREP_FACE_MERGE_RESEAM`), which joins two pieces of one cylinder or
    /// frustum split only along the carrier's seam, unless a reference that
    /// names something on the body as merged would name something else, or
    /// nothing, on the joined one; `None` never rejoins.
    #[serde(default)]
    pub seam_rejoin_later_references: Option<Vec<String>>,
}

thread_local! {
    static WITHOUT_PERTURBATION: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
}

/// Runs `action` with this thread's booleans refusing a degenerate exact
/// arrangement where they would retry it on rigidly perturbed operands (the
/// perturbation ladder, the last resort after the exact recoveries, which
/// still run): for a caller with a fallback of its own (another layout of the
/// same cut), which comes back with the ladder only where every fallback
/// refused.
pub fn without_perturbation<T>(action: impl FnOnce() -> T) -> T {
    struct Restore(bool);
    impl Drop for Restore {
        fn drop(&mut self) {
            WITHOUT_PERTURBATION.with(|cell| cell.set(self.0));
        }
    }
    let _restore = Restore(WITHOUT_PERTURBATION.with(|cell| cell.replace(true)));
    action()
}

/// [`BooleanOptions::touched_assembly`] under its environment override.
fn touched_assembly(options: &BooleanOptions) -> bool {
    match std::env::var("BREP_TOUCHED_ASSEMBLY").as_deref() {
        Ok("0") => false,
        Ok(_) => true,
        Err(_) => options.touched_assembly,
    }
}

/// [`BooleanOptions::disjoint_fast_path`] under its environment override.
fn disjoint_fast_path(options: &BooleanOptions) -> bool {
    match std::env::var("BREP_DISJOINT_FAST_PATH").as_deref() {
        Ok("0") => false,
        Ok(_) => true,
        Err(_) => options.disjoint_fast_path,
    }
}

/// [`BooleanOptions::contact_fast_path`] under its environment override.
fn contact_fast_path(options: &BooleanOptions) -> bool {
    match std::env::var("BREP_CONTACT_FAST_PATH").as_deref() {
        Ok("0") => false,
        Ok(_) => true,
        Err(_) => options.contact_fast_path,
    }
}

/// [`BooleanOptions::untouched_passthrough`] under its environment override.
fn untouched_passthrough(options: &BooleanOptions) -> bool {
    match std::env::var("BREP_UNTOUCHED_PASSTHROUGH").as_deref() {
        Ok("0") => false,
        Ok(_) => true,
        Err(_) => options.untouched_passthrough,
    }
}

fn default_tolerance() -> f64 {
    1e-7
}

fn default_true() -> bool {
    true
}

impl Default for BooleanOptions {
    fn default() -> Self {
        Self {
            tolerance: default_tolerance(),
            tolerances: None,
            imprint: ImprintOptions::default(),
            merge_coplanar_faces: true,
            keep_unmerged_name_substrs: Vec::new(),
            untouched_passthrough: true,
            disjoint_fast_path: true,
            contact_fast_path: true,
            touched_assembly: false,
            sector_classification: true,
            sheet_trim: true,
            keep_cells: false,
            seam_reseat_later_references: None,
            seam_rejoin_later_references: None,
        }
    }
}

mod select;
mod sheet;
mod cellular;
pub mod sector;
pub mod assemble;
mod operand_state;
mod rim;
mod coincident;
mod touch;
pub(crate) mod reseat;
mod ball_align;
mod rejoin;
mod window_retry;

pub use rim::*;
use select::*;
use coincident::*;
use touch::TouchCensus;
pub use assemble::{
    assemble_fragments, assemble_open_fragments, commit_nearby_edge_endpoints,
    clear_candidate_memo, edge_interior_lies_on, finalize_assembled_solid, refresh_derived_genus,
};
use assemble::{assemble_fragments_scoped, AssemblyScope};
// Currently referenced only inside `assemble`, so the re-export is unused.
#[allow(unused_imports)]
pub use assemble::apply_assembly_heal_chain;


/// A boolean's operands at the working scale (`brep/rescale.rs`): unless `BREP_WORKING_SCALE=0`, operands whose combined box diagonal
/// is below 1/16 are scaled up by a power of two, and the result is scaled back.
/// Under `BREP_LARGE_WORKING_SCALE`, a refused large operation also tries a
/// smaller working scale. Its successful original result always takes precedence.
struct WorkingScale {
    factor: f64,
    solids: Vec<BrepSolid>,
    options: BooleanOptions,
}

thread_local! {
    // The original attempt includes its nested boolean calls. Only after it
    // refuses do we try the large-model working scale; unwind restores it.
    static ORIGINAL_WORKING_SCALE: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
}

impl WorkingScale {
    fn of(operands: &[&BrepSolid], options: &BooleanOptions) -> Result<Option<Self>, KernelRefusal> {
        let Some(bounds) = crate::rescale::bounds_of(operands.iter().copied()) else {
            return Ok(None);
        };
        let Some(factor) = crate::rescale::guard_factor(bounds).or_else(|| {
            if ORIGINAL_WORKING_SCALE.with(|state| state.get()) { None }
            else { crate::rescale::large_guard_factor(bounds) }
        }) else {
            return Ok(None);
        };
        crate::rescale::profile_fired("boolean", factor, bounds);
        let solids = operands
            .iter()
            .map(|solid| crate::rescale::scale_operand(solid, factor, "boolean"))
            .collect::<Result<Vec<_>, _>>()?;
        // The one length the options carry in model units.
        let mut options = options.clone();
        options.imprint.maximum_ssi_step = options.imprint.maximum_ssi_step.map(|step| step * factor);
        Ok(Some(Self { factor, solids, options }))
    }

    /// Retain the original successful result bit for bit. A failed fallback
    /// retains the original refusal too, rather than replacing its evidence.
    fn run<T>(
        &self,
        original: impl FnOnce() -> Result<T, KernelRefusal>,
        scaled: impl FnOnce() -> Result<T, KernelRefusal>,
    ) -> Result<T, KernelRefusal> {
        if self.factor >= 1.0 { return scaled(); }
        struct Restore(bool);
        impl Drop for Restore {
            fn drop(&mut self) { ORIGINAL_WORKING_SCALE.with(|state| state.set(self.0)); }
        }
        let answer = {
            let _restore = Restore(ORIGINAL_WORKING_SCALE.with(|state| state.replace(true)));
            original()
        };
        match answer {
            Ok(value) => Ok(value),
            Err(original) => scaled().or(Err(original)),
        }
    }

    fn back(&self, solid: &BrepSolid) -> Result<BrepSolid, KernelRefusal> {
        crate::rescale::scale_operand(solid, 1.0 / self.factor, "boolean")
    }

    fn refusal(&self, error: KernelRefusal) -> KernelRefusal {
        crate::rescale::name_scale(error, self.factor)
    }
}

pub fn boolean_operation(
    first: &BrepSolid,
    second: &BrepSolid,
    operation: BooleanOperation,
    options: &BooleanOptions,
) -> Result<BrepSolid, KernelRefusal> {
    if let Some(scaled) = WorkingScale::of(&[first, second], options)? {
        return scaled.run(
            || boolean_operation(first, second, operation, options),
            || boolean_operation(&scaled.solids[0], &scaled.solids[1], operation, &scaled.options)
                .map_err(|error| scaled.refusal(error))
                .and_then(|solid| scaled.back(&solid)),
        );
    }
    boolean_operation_unscaled(first, second, operation, options)
        .or_else(|refused| window_retry::retry(first, second, operation, options, refused, boolean_operation_unscaled))
}

/// [`boolean_operation`] on operands already at their working scale.
fn boolean_operation_unscaled(
    first: &BrepSolid,
    second: &BrepSolid,
    operation: BooleanOperation,
    options: &BooleanOptions,
) -> Result<BrepSolid, KernelRefusal> {
    match boolean_operation_with_diagnostics(first, second, operation, options) {
        Ok(outcome) => {
            // An empty difference or common part of two solids whose faces only
            // nearly coincide is the near pair read as one, not the answer.
            if below_tolerance_refusal_enabled()
                && outcome.value.shells.iter().all(|shell| shell.faces.is_empty())
                && !matches!(operation, BooleanOperation::Union)
            {
                if let Some(refusal) = below_tolerance_refusal(first, second, None) {
                    if let Some(answer) = below_tolerance_answer(first, second, operation) {
                        return Ok(answer);
                    }
                    if let Some(answer) = below_tolerance_rescaled(first, second, operation, options) {
                        return Ok(answer);
                    }
                    return Err(refusal);
                }
            }
            if unresolved_merge_refusal_enabled() && !matches!(operation, BooleanOperation::Subtract) {
                if let Some(refusal) = unresolved_merge_refusal(first, second) {
                    return Err(refusal);
                }
            }
            // A union or common part the exact lane did write over a near pair
            // welds the slivers between the pair partly; the inclusion is exact.
            if below_tolerance_slivers_enabled()
                && !matches!(operation, BooleanOperation::Subtract)
                && [first, second].iter().all(|solid| face_count(solid) <= SLIVER_ANSWER_FACES)
                && near_coincident_planar_faces(first, second).is_some()
            {
                if let Some(answer) = below_tolerance_answer(first, second, operation) {
                    return Ok(answer);
                }
            }
            Ok(outcome.value)
        }
        Err(error) => {
            if below_tolerance_refusal_enabled() {
                if let Some(refusal) = below_tolerance_refusal(first, second, Some(&error)) {
                    if let Some(answer) = below_tolerance_answer(first, second, operation) {
                        return Ok(answer);
                    }
                    if let Some(answer) = below_tolerance_rescaled(first, second, operation, options) {
                        return Ok(answer);
                    }
                    return Err(refusal);
                }
            }
            // Perturbation-fallback lane (Simulation of Simplicity). The exact
            // arrangement only errs on a DEGENERACY here (coincident /
            // near-tangent carrier faces make it structurally inconsistent) —
            // e.g. a partial torus whose equatorial circles lie exactly in a box
            // face. Retry once on a rigidly perturbed second operand; accept only
            // an oracle-clean, volume-consistent result. See
            // `perturbation_retry`.
            if std::env::var("BREP_NO_PERTURB").as_deref() == Ok("1")
                || !error.class.perturbation_eligible()
            {
                return Err(error);
            }
            // OPERAND-CONSISTENCY GATE. A rigid nudge lifts a coincidence
            // BETWEEN the operands; it cannot reconcile an operand with
            // itself. A planar face whose straight boundary sits farther off
            // its plane than the assembly weld — the shape the STEP importer's
            // planar split repairs, so it reaches here only on a body built
            // some other way — fails the exact arrangement for every rung
            // alike: the 2026-09-12 mesh-import report paid twenty ~7 s rungs
            // for one 3.4e-3 gap before the import split existed. Name the
            // gap and stop. The predicate is deliberately the splitter's own,
            // not a pcurve-error band: vendor bodies whose fitted edges stand
            // a few 1e-4 off curved faces are legitimate input (per-entity
            // tolerances) and keep every rung.
            // OPERAND-CLOSURE GATE. A boolean of a body that is not a closed
            // shell has no solid answer, exact or nudged: its inside is not
            // defined, and its volume depends on the reference point. The
            // slimmed fuzz fixtures (13, 14 and 15, operand A cut from a larger
            // part by a box filter) carry 54, 97 and 66 edges that bound one
            // face; every one of their 20 rungs refused on the exact lane, and
            // 15's identity was read from an intersection with the open shell.
            // Only consulted on the failure path, so nothing that succeeds
            // today changes. Escape hatch: BREP_OPEN_OPERAND_GATE=0.
            if std::env::var("BREP_OPEN_OPERAND_GATE").as_deref() != Ok("0") {
                if let Some((operand, open)) = [first, second]
                    .iter()
                    .enumerate()
                    .find_map(|(operand, solid)| {
                        let open = one_use_edges(solid);
                        (open > 0).then_some((operand, open))
                    })
                {
                    return Err(error.with_message(|message| format!(
                        "{message}; operand {operand} is not a closed shell: {open} of its edges bound \
                         only one face, so no boolean of it encloses a solid and no perturbation \
                         can close it"
                    )));
                }
            }
            if let Some((operand, face_id, deviation, weld)) =
                planar_operand_off_its_plane(first, second, options)
            {
                return Err(error.with_message(|message| format!(
                    "{message}; operand {operand} face {face_id} is a plane whose straight \
                     boundary sits {deviation:.3e} off it (assembly weld {weld:.1e}), which \
                     no perturbation can reconcile — the body's planes must contain their \
                     boundary before it can take part in an exact boolean"
                )));
            }
            // INTERNAL-TANGENCY PINCH GATE. Perturbation is only legitimate
            // where the exact arrangement is UNSTABLE but its ANSWER is not:
            // the nearby transversal case must carry the same topology the
            // degenerate one does. An internal tangency in a DIFFERENCE breaks
            // exactly that premise — the exact `A − B` pinches to zero
            // thickness along the contact, so an epsilon either tears the wall
            // open (a through-slot where the exact answer has intact material)
            // or leaves a sub-micron web. Both are within epsilon in Hausdorff
            // distance and in volume, so neither the oracle (boundary skip
            // band) nor the 1% CSG volume bound can see them, and the lane
            // would ship the tolerant-kernel sliver this kernel refuses. Only
            // consulted on the failure path, so nothing that succeeds today can
            // change; the honest exact-path error is returned instead.
            // Escape hatch for tamper-verification: BREP_PINCH_GATE=0.
            if matches!(operation, BooleanOperation::Subtract)
                && std::env::var("BREP_PINCH_GATE").as_deref() != Ok("0")
                && subtract_pinches_at_internal_tangency(first, second, options.tolerance)
                    .unwrap_or(false)
            {
                // The exact answer exists where the contact only touches: see
                // `subtract_through_the_common_part`. Not a perturbation.
                if let Some(solid) = subtract_through_the_common_part(first, second, options) {
                    return Ok(solid);
                }
                return Err(error.with_message(|message| format!(
                    "{message}; internal tangency: the operands touch tangentially with \
                     co-directed normals, so the exact difference pinches to zero thickness \
                     (non-manifold, unrepresentable in a boundary model) — refusing rather \
                     than returning a perturbed sliver"
                )));
            }
            // TANGENT-NODE GATE, same doctrine as the pinch gate above and for
            // the same reason. A 4-valent tangent node is refused because the
            // kernel cannot imprint it, and perturbation is only legitimate
            // where the exact arrangement is UNSTABLE but its ANSWER is not:
            // the nudged configuration resolves the kiss one way or the other
            // and the kernel has no claim on which. Until 2026-09-13 the ladder
            // could not produce a candidate here at all; with `fit_polyline`
            // measuring its own result the equal-radius torus pair's LAST rung
            // (3.0e-3 of scale) assembles, validates, and passes the semantic
            // oracle at 399 samples with a zero disagreement rate, reading
            // 299.4998500 against the ideal union's 299.6 — a 0.03% error that
            // is the nudge's, not the kernel's answer. Shipping that would
            // answer a question this class exists to decline, so the class does
            // not take the ladder. Escape hatch for measuring it:
            // BREP_TANGENT_NODE_GATE=0.
            if matches!(error.class, RefusalClass::TangentNodeSingularity)
                && std::env::var("BREP_TANGENT_NODE_GATE").as_deref() != Ok("0")
            {
                return Err(error);
            }
            // The union of two bodies tangent inside one another has an exact
            // answer the arrangement of the difference reaches: see
            // `union_through_the_difference`. Not a perturbation.
            if matches!(operation, BooleanOperation::Union) {
                if let Some(solid) = union_through_the_difference(first, second, options) {
                    return Ok(solid);
                }
            }
            // The ladder itself: not under `without_perturbation`.
            if WITHOUT_PERTURBATION.with(|cell| cell.get()) {
                return Err(error);
            }
            match perturbation_retry(first, second, operation, options) {
                Ok(solid) => Ok(solid),
                Err(refused) if refused.is_empty() => Err(error),
                Err(refused) => {
                    Err(error.with_message(|message| format!("{message}; {}", refused.join("; "))))
                }
            }
        }
    }
}

/// Grid resolution (per parameter direction, endpoints included) used to probe a
/// face pair for a tangential contact. Endpoints and the midpoint are both
/// sampled, so a contact sitting on a periodic seam (a cylinder's `u = 0`
/// ruling) and one sitting at a face's parametric centre are both hit exactly.
const PINCH_PROBE_STEPS: usize = 8;

/// `|n_a × n_b|` bound below which two unit normals count as parallel — the same
/// bound `csg::imprint`'s pair classifier uses (`PAIR_ANGULAR_TOLERANCE`).
const PINCH_ANGULAR_TOLERANCE: f64 = 1e-4;

/// Parameter-space step (fraction of the domain span) taken around a contact
/// sample to prove the contact is LOWER-DIMENSIONAL — a tangency curve/point
/// rather than a cosurface patch.
const PINCH_SEPARATION_STEP: f64 = 1e-2;

/// `first − second` as `first − (first ∩ second)`, on the exact lane only, for a
/// difference the pinch gate is about to refuse (`BREP_PINCH_COMMON_RETRY`,
/// default on; `=0` leaves the refusal).
///
/// The two are the same set, and the second form is often the arrangement the
/// exact lane can build where the first is not. A cylinder cut by a smaller one
/// tangent to it inside along a ruling, the cutter running on past both caps
/// (a crescent; brep-bench probe 2026-11-12): each cap's section circle is
/// tangent to the cap's own rim, the cap's loops do not close, and the cut
/// refused; with the cutter flush with the caps it built, exact. The common part
/// IS the cutter made flush, it is built without trouble (it has no thin wall),
/// and cutting it away meets the first operand only in faces they share.
///
/// Nothing is nudged, so the pinch gate's reason for refusing a PERTURBED
/// answer does not apply; what is accepted is an exact-lane body that validates
/// and whose volume is `V(first) − V(first ∩ second)` to the drift bar. Any
/// other outcome returns `None` and the gate's refusal stands.
fn subtract_through_the_common_part(first: &BrepSolid, second: &BrepSolid, options: &BooleanOptions) -> Option<BrepSolid> {
    // With the tangent imprint switched off the caller has asked for the
    // recorded-only lane, whose answer to a pinch is its refusal.
    if !switch!("BREP_PINCH_COMMON_RETRY", true) || !options.imprint.tangent_curve_imprint {
        return None;
    }
    let common = boolean_operation_with_diagnostics(first, second, BooleanOperation::Intersect, options).ok()?.value;
    if common.shells.iter().all(|shell| shell.faces.is_empty()) {
        return None;
    }
    let cut = boolean_operation_with_diagnostics(first, &common, BooleanOperation::Subtract, options).ok()?.value;
    if !cut.validate().is_empty() {
        return None;
    }
    let volume = |solid: &BrepSolid| crate::solid_mass_properties(solid).ok().map(|mass| mass.volume);
    let (whole, taken, left) = (volume(first)?, volume(&common)?, volume(&cut)?);
    ((left - (whole - taken)).abs() <= 1e-9 * whole.abs().max(1.0)).then_some(cut)
}

/// `first ∪ second` as `second ∪ (first − second)`, on the exact lane only, for
/// a union that failed where the operands touch tangentially with co-directed
/// normals (`BREP_TANGENT_UNION_DIFFERENCE_RETRY`, default on; `=0` leaves the
/// union to the paths it had).
///
/// The two are the same set. A cylinder holding a smaller one tangent to it
/// inside along a ruling, the smaller running on past a cap: the union's
/// arrangement leaves the cap's section circle tangent to the cap's own rim and
/// the shell does not close, while the difference builds (the crescent) and the
/// smaller body joins it along faces the two share. Accepted only when it
/// validates and its volume is `V(first − second) + V(second)` to the drift bar;
/// any other outcome returns `None` and the union goes on as before.
fn union_through_the_difference(first: &BrepSolid, second: &BrepSolid, options: &BooleanOptions) -> Option<BrepSolid> {
    if !switch!("BREP_TANGENT_UNION_DIFFERENCE_RETRY", true) || !options.imprint.tangent_curve_imprint {
        return None;
    }
    let pinches = |a: &BrepSolid, b: &BrepSolid| subtract_pinches_at_internal_tangency(a, b, options.tolerance).unwrap_or(false);
    if !pinches(first, second) && !pinches(second, first) {
        return None;
    }
    let difference = match boolean_operation_with_diagnostics(first, second, BooleanOperation::Subtract, options) {
        Ok(outcome) => outcome.value,
        Err(_) => subtract_through_the_common_part(first, second, options)?,
    };
    if difference.shells.iter().all(|shell| shell.faces.is_empty()) || !difference.validate().is_empty() {
        return None;
    }
    let union = boolean_operation_with_diagnostics(second, &difference, BooleanOperation::Union, options).ok()?.value;
    if !union.validate().is_empty() {
        return None;
    }
    let volume = |solid: &BrepSolid| crate::solid_mass_properties(solid).ok().map(|mass| mass.volume);
    let (left, added, whole) = (volume(&difference)?, volume(second)?, volume(&union)?);
    ((whole - (left + added)).abs() <= 1e-9 * whole.abs().max(1.0)).then_some(union)
}

/// Would the exact `first − second` PINCH to zero thickness at a tangential
/// contact?
///
/// True when some face of `first` and some face of `second` touch tangentially
/// (surfaces within the contact band, normals parallel) with CO-DIRECTED outward
/// normals, at a contact that is lower-dimensional (the surfaces separate as you
/// step away from it).
///
/// Co-directed outward normals at a tangency mean one solid lies locally INSIDE
/// the other: subtracting leaves material on both sides of the contact that
/// meets there at zero thickness — a non-manifold pinch no boundary model can
/// represent. Anti-directed normals are the harmless EXTERNAL tangency (a
/// cylinder resting against a wall): the difference simply keeps `first` intact.
/// The lower-dimensionality requirement excludes a CO-SURFACE contact (a pocket
/// wall flush with an outer wall — co-directed normals, but the surfaces stay
/// coincident in every direction), where the difference is perfectly well
/// behaved.
///
/// Conservative by construction: it only reports a pinch it can actually witness
/// on the probe grid, and a miss just leaves the perturbation lane to its
/// existing gates.
pub fn subtract_pinches_at_internal_tangency(
    first: &BrepSolid,
    second: &BrepSolid,
    tolerance: f64,
) -> Result<bool, KernelRefusal> {
    // Same contact band the imprint pair classifier uses for its sampled
    // "these carriers touch" verdict.
    let contact = (tolerance * 100.0).max(1e-12);
    let probes_a = first
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .map(PinchProbe::of)
        .collect::<Result<Vec<_>, _>>()?;
    let probes_b = second
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .map(PinchProbe::of)
        .collect::<Result<Vec<_>, _>>()?;
    for probe_a in &probes_a {
        for probe_b in &probes_b {
            // Sampled-hull cull, so the (failure-path-only) cost stays linear in
            // the faces that actually touch rather than quadratic in every face.
            // The hull is built from the probe grid, so it can under-cover a
            // curved carrier between samples: pad it by a percent of the pair's
            // size before culling. Over-keeping a pair only costs a probe that
            // finds nothing.
            let pad = 1e-2 * probe_a.extent().max(probe_b.extent());
            if probe_a.separation(probe_b) > contact + pad {
                continue;
            }
            // CO-SURFACE contacts are 2-dimensional, not tangencies: two flush
            // walls (a pocket's side coincident with an outer wall, a cylinder
            // cap lying in a box face) have co-directed normals wherever the
            // solids nest, yet the difference there is perfectly well behaved.
            // Only a LOWER-dimensional contact pinches, so drop the pair the
            // pair classifier calls cosurface before probing it.
            if classify_surface_pair(
                &probe_a.face.surface,
                &probe_b.face.surface,
                tolerance,
                PINCH_ANGULAR_TOLERANCE,
            ).or_refuse(KernelStage::Validate, "csg.boolean.mod")?
            .relation
                == SurfacePairRelation::Cosurface
            {
                continue;
            }
            if faces_touch_with_codirected_normals(probe_a, probe_b, contact)? {
                return Ok(true);
            }
        }
    }
    Ok(false)
}

/// A face's probe grid plus the grid's bounding box, built once per face so the
/// pair loop only pays for a hull comparison.
struct PinchProbe<'a> {
    face: &'a FaceRecord,
    /// `(u, v, point)` on the face's carrier surface.
    samples: Vec<(f64, f64, Vec3)>,
    minimum: [f64; 3],
    maximum: [f64; 3],
}

impl<'a> PinchProbe<'a> {
    fn of(face: &'a FaceRecord) -> Result<Self, KernelRefusal> {
        let [u0, u1] = face.surface.domain_u().or_refuse(KernelStage::Validate, "domain_u")?;
        let [v0, v1] = face.surface.domain_v().or_refuse(KernelStage::Validate, "domain_v")?;
        let steps = PINCH_PROBE_STEPS as f64;
        let mut samples = Vec::with_capacity((PINCH_PROBE_STEPS + 1).pow(2));
        let mut minimum = [f64::INFINITY; 3];
        let mut maximum = [f64::NEG_INFINITY; 3];
        for i in 0..=PINCH_PROBE_STEPS {
            let u = u0 + (u1 - u0) * i as f64 / steps;
            for j in 0..=PINCH_PROBE_STEPS {
                let v = v0 + (v1 - v0) * j as f64 / steps;
                let Ok(point) = face.surface.evaluate(u, v) else {
                    continue;
                };
                for (axis, value) in [point.x, point.y, point.z].into_iter().enumerate() {
                    minimum[axis] = minimum[axis].min(value);
                    maximum[axis] = maximum[axis].max(value);
                }
                samples.push((u, v, point));
            }
        }
        Ok(Self {
            face,
            samples,
            minimum,
            maximum,
        })
    }

    /// Largest side of the sampled hull (0 when nothing sampled).
    fn extent(&self) -> f64 {
        (0..3)
            .map(|axis| self.maximum[axis] - self.minimum[axis])
            .fold(0.0f64, f64::max)
    }

    /// Axis-aligned gap between the two sampled hulls (0 when they overlap).
    fn separation(&self, other: &Self) -> f64 {
        let mut gap: f64 = 0.0;
        for axis in 0..3 {
            gap = gap.max(self.minimum[axis] - other.maximum[axis]);
            gap = gap.max(other.minimum[axis] - self.maximum[axis]);
        }
        gap
    }
}

/// One face pair of [`subtract_pinches_at_internal_tangency`]: probe both
/// surfaces on a grid, keep samples that land on the other surface inside BOTH
/// trims with parallel co-directed outward normals, and accept only where the
/// contact provably separates nearby.
fn faces_touch_with_codirected_normals(
    probe_a: &PinchProbe<'_>,
    probe_b: &PinchProbe<'_>,
    contact: f64,
) -> Result<bool, KernelRefusal> {
    for (probe, other) in [(probe_a, probe_b), (probe_b, probe_a)] {
        let source = probe.face;
        let target = other.face;
        for &(u, v, point) in &probe.samples {
            let projection = project_point_to_surface(&target.surface, point).or_refuse(KernelStage::Validate, "project_point_to_surface")?;
            if projection.distance > contact {
                continue;
            }
            // A pole (collapsed du × dv) has no reliable orientation here;
            // skipping it keeps the predicate conservative.
            let (Ok(source_normal), Ok(target_normal)) = (
                source.surface.normal(u, v),
                target.surface.normal(projection.u, projection.v),
            ) else {
                continue;
            };
            let source_outward = outward_normal(source_normal, source.same_sense);
            let target_outward = outward_normal(target_normal, target.same_sense);
            if source_outward.cross(target_outward).length() > PINCH_ANGULAR_TOLERANCE
                || source_outward.dot(target_outward) <= 0.0
            {
                continue;
            }
            if parameter_point_in_face(source, Vec2 { x: u, y: v }, 1e-6).or_refuse(KernelStage::Validate, "parameter_point_in_face")?
                == PolygonClass::Outside
                || parameter_point_in_face(
                    target,
                    Vec2 {
                        x: projection.u,
                        y: projection.v,
                    },
                    1e-6,
                ).or_refuse(KernelStage::Validate, "csg.boolean.mod")? == PolygonClass::Outside
            {
                continue;
            }
            if contact_separates_locally(source, target, u, v, contact)? {
                return Ok(true);
            }
        }
    }
    Ok(false)
}

fn outward_normal(normal: Vec3, same_sense: bool) -> Vec3 {
    if same_sense {
        normal
    } else {
        normal.scale(-1.0)
    }
}

/// Does the contact at `source(u, v)` LEAVE the contact band when you step away
/// from it in parameter space? A tangency curve or point does (stepping across
/// the tangency separates the surfaces quadratically); a cosurface patch does
/// not (it stays coincident in every direction). Steps are clamped into the
/// domain, so a sample on a seam or a domain edge probes only inward.
fn contact_separates_locally(
    source: &FaceRecord,
    target: &FaceRecord,
    u: f64,
    v: f64,
    contact: f64,
) -> Result<bool, KernelRefusal> {
    let [u0, u1] = source.surface.domain_u().or_refuse(KernelStage::Validate, "domain_u")?;
    let [v0, v1] = source.surface.domain_v().or_refuse(KernelStage::Validate, "domain_v")?;
    let step_u = (u1 - u0) * PINCH_SEPARATION_STEP;
    let step_v = (v1 - v0) * PINCH_SEPARATION_STEP;
    for (probe_u, probe_v) in [
        ((u + step_u).min(u1), v),
        ((u - step_u).max(u0), v),
        (u, (v + step_v).min(v1)),
        (u, (v - step_v).max(v0)),
    ] {
        if (probe_u - u).abs() < f64::EPSILON && (probe_v - v).abs() < f64::EPSILON {
            continue;
        }
        let Ok(point) = source.surface.evaluate(probe_u, probe_v) else {
            continue;
        };
        // A generous multiple of the band: the step must clear it decisively,
        // never on projection noise.
        if project_point_to_surface(&target.surface, point).or_refuse(KernelStage::Validate, "project_point_to_surface")?.distance > contact * 10.0 {
            return Ok(true);
        }
    }
    Ok(false)
}

/// Deterministic perturbation-fallback (Simulation of Simplicity), the textbook
/// CAD answer to an exact/near-tangent arrangement degeneracy, adopted after six
/// incremental arrangement/imprint/fragment fixes for the near-tangent class were
/// each falsified, rather than attempting a seventh. Rigidly TRANSLATE the second operand by a tiny
/// epsilon so the degenerate contact becomes a clean transversal crossing, redo
/// the boolean, and accept the result ONLY if it is (a) topologically valid —
/// guaranteed by `assemble`'s own gate returning `Ok` — (b) semantically equal to
/// the exact CSG of the ORIGINAL operands (the point-classification oracle), and
/// (c) volume-consistent with the CSG inequality bounds. A wrong perturbed result
/// is rejected on (b)/(c), so this lane preserves the engine's fail-safe property:
/// it can only turn a degeneracy-error into a CORRECT solid, never a wrong one.
///
/// INVARIANT — no recursion: this calls `boolean_operation_with_diagnostics`
/// (the EXACT path), never `boolean_operation`, so the fallback can never
/// re-enter itself. A future refactor must preserve that.
///
/// (d) THE IDENTITY. Where the exact lane answers one of the other operations
/// on the same operands, inclusion–exclusion names the volume this result
/// must have, and a rung that misses it by more than the case gate's own drift
/// band (`VOLUME_DRIFT_REL` of that volume, `VOLUME_DRIFT_ABS` floor) is
/// refused by name. (c)'s 1% of the operands' total cannot see an error in a
/// small removed volume: on the helmet's cutter − helmet (2026-09-15) the rung
/// at fraction 3e-4 moved the helmet 5.2e-3, 2.3% of its size, passed the
/// oracle at 1 in 336, and read 999.999899266 against V(cutter) − V(cutter ∩
/// helmet) = 999.999905792: 6.9% too much material removed. Where no other
/// operation clears on the exact lane there is no identity to hold, and the
/// rung is held to (e) instead. Escape hatch for measuring it:
/// BREP_PERTURB_IDENTITY=0 (which also lifts (e)).
///
/// (e) THE MIRROR. Where no exact-lane boolean of the operands clears in
/// either order, the rung is held to itself moved the other way: the same
/// operation, or failing that any sibling reading, on the second operand moved
/// by −offset, with V(first) and V(second) from the mass integrator and the
/// operation's sign, names the volume the mirrored rung encloses. A rigid
/// translation changes the answer to first order in the offset, so half the
/// difference between the rung's body and its mirror is the body's
/// first-order translation error, and it must clear the same drift band. A
/// rung whose mirror clears no boolean at all has nothing to check it and is
/// refused by name. On 22_sag_boundary_identity_t164 (2026-09-15) the four
/// shipped rescues at offset 9.7e-4 disagreed with each other through V(∩) by
/// 4.3e-2 (the difference moves the part; the others move the sphere), against
/// a bar of 5e-6. Escape hatch: BREP_PERTURB_MIRROR=0.
///
/// (d) and (e) run before (b), the oracle, which costs more than either.
///
/// (f) THE EARLY EXIT. Along one direction the mirror's error grows with the
/// offset (`MirrorTrend` states the law and its measurement), so a direction
/// whose two refused rungs read one sign, growing, is not tried at larger
/// offsets, and the refusal names it. booleanIssue's refusal walked 20 rungs,
/// each with its mirror, in 40.5 s against the case's 30 s. Escape hatch:
/// BREP_MIRROR_EARLY_EXIT=0.
///
/// `Err` carries one sentence per rung refused on (d), empty when none was.
fn perturbation_retry(
    first: &BrepSolid,
    second: &BrepSolid,
    operation: BooleanOperation,
    options: &BooleanOptions,
) -> Result<BrepSolid, Vec<String>> {
    // Generic translation directions: each has a substantial component on ALL
    // three axes, so whatever axis the coincidence normal lies along (an
    // equatorial plane flush with a box face, a cap plane flush with a side
    // face, …) at least one direction has a component that lifts the contact off
    // exact tangency. Axis-aligned directions are deliberately excluded — a nudge
    // parallel to the coincident plane leaves the degeneracy in place (verified:
    // pure ±X / ±Z never rescue the flush-torus case). Fixed, not hash-seeded
    // as the lane was first sketched: the seeding's only purpose was to rule out
    // RNG/clock nondeterminism, which fixed vectors already satisfy, and four
    // diverse directions cover any axis-aligned normal.
    const DIRECTIONS: [[f64; 3]; 4] = [
        [0.4034, 0.7973, 0.4491],
        [0.7973, 0.4491, 0.4034],
        [0.4491, 0.4034, 0.7973],
        [0.5774, -0.5774, 0.5774],
    ];
    // Fractions of operand scale, ordered SMALLEST-FIRST so the accepted solid
    // carries the least geometric offset. Sized above the near-tangent sliver
    // dead zone and below local feature size; the ladder + accept-gate make the
    // exact magnitude non-critical (a rung that lands in a dead zone simply fails
    // the gate and the next rung is tried).
    const FRACTIONS: [f64; 5] = PERTURBATION_FRACTIONS;

    let debug = std::env::var("BREP_DEBUG_BOOL").is_ok();
    let scale = crate::solid_tolerance::solid_scale(first)
        .max(crate::solid_tolerance::solid_scale(second))
        .max(1.0);

    // Original operand volumes for the CSG inequality backstop (the oracle skips
    // points near boundaries, so a wrong thin shell hugging the flush face is
    // invisible to it but shows up here).
    let _caller = crate::mass_caller("boolean.perturbation_rescue");
    let va = solid_signed_volume(first).ok().map(f64::abs);
    let vb = solid_signed_volume(second).ok().map(f64::abs);
    let hold_identity = std::env::var("BREP_PERTURB_IDENTITY").as_deref() != Ok("0");
    // (e) is held only with (d): `BREP_PERTURB_MIRROR=0` alone ships a rung
    // with no identity on (a)–(c), as before.
    let hold_mirror = hold_identity && std::env::var("BREP_PERTURB_MIRROR").as_deref() != Ok("0");
    // (f) A direction whose mirror errors say no larger offset can clear is
    // closed. Escape hatch: `BREP_MIRROR_EARLY_EXIT=0` walks every rung.
    let early_exit = std::env::var("BREP_MIRROR_EARLY_EXIT").as_deref() != Ok("0");
    let mut trends = [MirrorTrend::default(); 4];
    // Measured once, on the first rung that clears (a)–(c).
    let mut identity: Option<Option<InclusionExclusion>> = None;
    let mut refused = Vec::new();
    // `BREP_DEBUG_PERTURB_LADDER=1`: every rung's disposition on stderr, and
    // for every rung that builds, its miss against the identity, measured
    // before the ladder starts. Changes no decision.
    let ladder = std::env::var("BREP_DEBUG_PERTURB_LADDER").is_ok();
    let ladder_start = web_time::Instant::now();
    if ladder {
        let measured = inclusion_exclusion_volume(first, second, operation, options, va, vb, false);
        match &measured {
            Some(identity) => eprintln!(
                "[ladder] identity {} = {:.9} from the exact lane's {} ({:.9}); V(first) {:?} V(second) {:?}",
                identity.formula, identity.expected, identity.reading, identity.read, va, vb
            ),
            None => eprintln!("[ladder] no identity: no exact-lane boolean of these operands clears in either order"),
        }
        identity = Some(measured);
    }
    let rung_miss = |identity: &Option<Option<InclusionExclusion>>, candidate: &BrepSolid| -> String {
        let volume = body_volume(candidate);
        match (identity, volume) {
            (Some(Some(identity)), Some(volume)) => format!(
                "volume {volume:.9} miss {:+.3e} bar {:.3e}",
                volume - identity.expected,
                crate::VOLUME_DRIFT_ABS.max(crate::VOLUME_DRIFT_REL * identity.expected.abs())
            ),
            (_, Some(volume)) => format!("volume {volume:.9}"),
            _ => "no volume".to_string(),
        }
    };

    for &fraction in &FRACTIONS {
        let magnitude = scale * fraction;
        for (direction, dir) in DIRECTIONS.iter().enumerate() {
            if trends[direction].closed {
                if ladder {
                    eprintln!("[ladder] dir={dir:?} fraction={fraction:.1e} offset {magnitude:.3e}: not tried, its direction is closed");
                }
                continue;
            }
            let offset = [dir[0] * magnitude, dir[1] * magnitude, dir[2] * magnitude];
            let translate = match AffineTransform::new([
                1.0, 0.0, 0.0, offset[0], //
                0.0, 1.0, 0.0, offset[1], //
                0.0, 0.0, 1.0, offset[2], //
                0.0, 0.0, 0.0, 1.0,
            ]) {
                Ok(transform) => transform,
                Err(_) => continue,
            };
            let moved = match crate::transform_brep(second, translate, false) {
                Ok(solid) => solid,
                Err(_) => continue,
            };
            let rung = format!("dir={dir:?} fraction={fraction:.1e} offset {magnitude:.3e}");
            let rung_start = web_time::Instant::now();
            let candidate =
                match boolean_operation_with_diagnostics(first, &moved, operation, options) {
                    Ok(outcome) => outcome.value,
                    Err(error) => {
                        if ladder {
                            eprintln!("[ladder] {rung}: exact lane refused {:?}", error.class);
                        }
                        continue;
                    }
                };
            let built = rung_start.elapsed();
            // A perturbation must not "rescue" a case into emptiness.
            if candidate.shells.is_empty() {
                if ladder {
                    eprintln!("[ladder] {rung}: empty");
                }
                continue;
            }
            // (c) Volume must satisfy the CSG inequality for the ORIGINAL
            // operands (generous 1% relative slack: catches gross wrongness —
            // doubled/halved bodies, missing lobes — without rejecting the
            // legitimate sub-micron offset).
            if let (Some(va), Some(vb)) = (va, vb) {
                if let Ok(vr) = solid_signed_volume(&candidate) {
                    if !volume_within_csg_bounds(operation, va, vb, vr.abs()) {
                        if ladder {
                            eprintln!("[ladder] {rung}: outside the CSG bound, {}", rung_miss(&identity, &candidate));
                        }
                        continue;
                    }
                }
            }
            // (d) and (e) run before (b): each costs a volume, and (e) one
            // exact-lane boolean, where the oracle classifies 400 points. A
            // rung ships only when every gate clears, so the order decides
            // which gate names a refusal, never which rung ships.
            let bounded = rung_start.elapsed();
            // (d) The exact lane's inclusion–exclusion identity.
            if hold_identity {
                let identity = identity.get_or_insert_with(|| {
                    let identity =
                        inclusion_exclusion_volume(first, second, operation, options, va, vb, false);
                    if debug && identity.is_none() {
                        eprintln!(
                            "[perturb] no exact-lane boolean of these operands clears in either \
                             order; no identity to hold"
                        );
                    }
                    identity
                });
                if let Some(identity) = identity {
                    let expected = identity.expected;
                    let bar = crate::VOLUME_DRIFT_ABS.max(crate::VOLUME_DRIFT_REL * expected.abs());
                    let volume = body_volume(&candidate);
                    let miss = volume.map(|volume| (volume - expected).abs());
                    if !miss.is_some_and(|miss| miss <= bar) {
                        let sentence = format!(
                            "perturbation rung dir={dir:?} fraction={fraction:.1e} \
                             (offset {magnitude:.3e}) refused: its body reads {} against the \
                             inclusion–exclusion identity {} = {expected:.9} (the exact lane's \
                             {} reads {:.9}), a miss of {} beyond the volume drift bar {bar:.3e}",
                            volume.map_or("no volume".to_string(), |volume| format!("{volume:.9}")),
                            identity.formula,
                            identity.reading,
                            identity.read,
                            miss.map_or("unmeasured".to_string(), |miss| format!("{miss:.3e}")),
                        );
                        if debug {
                            eprintln!("[perturb] {sentence}");
                        }
                        if ladder {
                            eprintln!(
                                "[ladder] {rung}: refused by the identity, volume {} miss {} bar {bar:.3e}",
                                volume.map_or("none".to_string(), |volume| format!("{volume:.9}")),
                                volume.map_or("unmeasured".to_string(), |volume| format!("{:+.3e}", volume - expected)),
                            );
                        }
                        refused.push(sentence);
                        continue;
                    }
                } else if hold_mirror {
                    // (e) No exact reading of these operands: the rung's
                    // mirror.
                    let mirror_start = web_time::Instant::now();
                    let mirrored = AffineTransform::new([
                        1.0, 0.0, 0.0, -offset[0], //
                        0.0, 1.0, 0.0, -offset[1], //
                        0.0, 0.0, 1.0, -offset[2], //
                        0.0, 0.0, 0.0, 1.0,
                    ])
                    .ok()
                    .and_then(|transform| crate::transform_brep(second, transform, false).ok());
                    let mirror = mirrored.as_ref().and_then(|mirrored| {
                        inclusion_exclusion_volume(first, mirrored, operation, options, va, vb, true)
                    });
                    let mirror_time = mirror_start.elapsed();
                    let volume = body_volume(&candidate);
                    let mut measured = None;
                    let verdict = match (&mirror, volume) {
                        (Some(mirror), Some(volume)) => {
                            let bar = crate::VOLUME_DRIFT_ABS.max(crate::VOLUME_DRIFT_REL * mirror.expected.abs());
                            let miss = 0.5 * (volume - mirror.expected).abs();
                            measured = Some((0.5 * (volume - mirror.expected), bar));
                            if ladder {
                                eprintln!(
                                    "[ladder] {rung}: mirror at −offset reads {} = {:.9} (the mirrored {} {:.9}); body {volume:.9}, first-order translation error {:+.3e} bar {bar:.3e}",
                                    mirror.formula, mirror.expected, mirror.reading, mirror.read, 0.5 * (volume - mirror.expected)
                                );
                            }
                            (miss > bar).then(|| format!(
                                "perturbation rung dir={dir:?} fraction={fraction:.1e} \
                                 (offset {magnitude:.3e}) refused: no exact-lane boolean of the \
                                 operands clears, so the rung is held to its mirror: its body reads \
                                 {volume:.9} with the second operand moved +offset and {} = {:.9} \
                                 with it moved −offset (the mirrored {} reads {:.9}), a first-order \
                                 translation error of {miss:.3e} beyond the volume drift bar {bar:.3e}",
                                mirror.formula, mirror.expected, mirror.reading, mirror.read
                            ))
                        }
                        _ => Some(format!(
                            "perturbation rung dir={dir:?} fraction={fraction:.1e} \
                             (offset {magnitude:.3e}) refused: no exact-lane boolean of the operands \
                             clears, and none clears with the second operand moved −offset either, \
                             so nothing checks the rung's body{}",
                            volume.map_or(String::new(), |volume| format!(" ({volume:.9})"))
                        )),
                    };
                    match verdict {
                        None => trends[direction].cleared(),
                        Some(sentence) => {
                            if debug {
                                eprintln!("[perturb] {sentence}");
                            }
                            if ladder {
                                eprintln!(
                                    "[ladder] {rung}: refused by its mirror (rung {:.0} ms: built {:.0}, bound {:.0}, mirror {:.0} ms; ladder {:.1} s)",
                                    rung_start.elapsed().as_secs_f64() * 1e3,
                                    built.as_secs_f64() * 1e3,
                                    (bounded - built).as_secs_f64() * 1e3,
                                    mirror_time.as_secs_f64() * 1e3,
                                    ladder_start.elapsed().as_secs_f64()
                                );
                            }
                            refused.push(sentence);
                            // (f) Two refused rungs of one sign, growing, close
                            // the direction.
                            let closure = measured.and_then(|(error, bar)| {
                                let earlier = trends[direction].refused(magnitude, error, early_exit)?;
                                Some((earlier, error, bar))
                            });
                            if let Some(((earlier_offset, earlier_error), error, bar)) = closure {
                                let closed = format!(
                                    "perturbation direction dir={dir:?} closed after offset {magnitude:.3e}: its \
                                     rungs' first-order translation errors read {earlier_error:+.3e} at offset \
                                     {earlier_offset:.3e} and {error:+.3e} at {magnitude:.3e}, one sign and \
                                     growing, so no larger offset along it comes back to the volume drift bar \
                                     {bar:.3e}"
                                );
                                if debug {
                                    eprintln!("[perturb] {closed}");
                                }
                                if ladder {
                                    eprintln!("[ladder] {closed}");
                                }
                                refused.push(closed);
                            }
                            continue;
                        }
                    }
                }
            }
            // (b) Semantic agreement with the exact CSG of the ORIGINAL
            // operands. Reject a vacuous verdict (skip-band swallowed every
            // sample so nothing was actually checked).
            match crate::oracle::boolean_semantic_disagreement(
                first, second, operation, &candidate, 400,
            ) {
                Ok(report) if report.considered >= 30 && !report.is_flagged() => {
                    if ladder {
                        eprintln!(
                            "[ladder] {rung}: accepted (oracle {} considered, rate {:.4}), {}",
                            report.considered, report.disagreement_rate, rung_miss(&identity, &candidate)
                        );
                    }
                    if debug {
                        eprintln!(
                            "[perturb] rescued {operation:?}: dir={dir:?} fraction={fraction:.1e} \
                             magnitude={magnitude:.3e} offset={offset:?} \
                             (oracle considered={} rate={:.4})",
                            report.considered, report.disagreement_rate
                        );
                    }
                    return Ok(candidate);
                }
                report => {
                    if ladder {
                        let verdict = match &report {
                            Ok(report) => format!("oracle {} considered, rate {:.4}", report.considered, report.disagreement_rate),
                            Err(error) => format!("oracle error {error:?}"),
                        };
                        eprintln!("[ladder] {rung}: refused by the oracle ({verdict}), {}", rung_miss(&identity, &candidate));
                    }
                    continue;
                }
            }
        }
    }
    if debug {
        eprintln!("[perturb] ladder exhausted for {operation:?}; no rung produced a clean result");
    }
    Err(refused)
}

/// The perturbation ladder's offsets, as fractions of the operands' scale,
/// smallest first.
pub const PERTURBATION_FRACTIONS: [f64; 5] = [3e-5, 1e-4, 3e-4, 1e-3, 3e-3];

/// (f) One ladder direction's rungs refused by their mirrors.
///
/// Along one direction a rung's signed first-order translation error, half
/// its body less its mirror's reading, reads s(δ) = kδ + r: a slope k from the
/// translation, and a remainder r from the degeneracy the rung steps over,
/// which does not grow with the offset. Two refused rungs at δ₁ < δ₂ with one
/// sign and |s(δ₂)| ≥ |s(δ₁)| fix k's sign as theirs, so every larger offset
/// reads farther from zero than s(δ₂), which is already beyond the drift bar:
/// the direction is closed. Rungs of one sign that shrink toward zero, or that
/// change sign, say nothing about k, and the direction stays open; a rung whose
/// mirror clears starts the direction's record again.
///
/// Measured 2026-09-15 (`BREP_DEBUG_PERTURB_LADDER=1` prints s). On
/// 22_sag_boundary_identity_t164, all four operations and all four
/// directions, s/δ keeps one sign and holds to 3.6% over the ladder's 100× of
/// offset (r ≈ 0; −22.23 .. −22.34 on the intersection's first direction). On
/// booleanIssue's flush partial torus union, s/δ holds to 0.5% and 8.5% on two
/// directions, and to 1.2% on a third past its smallest rung, whose remainder
/// reads −43.4 against the slope's −24.5. The first direction's smallest rung
/// reads −1.891e-2 where its slope, +2.3 .. +2.6, gives +3.5e-3 (r ≈ −2.2e-2),
/// and its next measured rung +3.510e-2: a change of sign, which keeps it open.
/// A closed direction's skipped rung would have had to fall to the bar, 8.3e2 ..
/// 1.5e6 times below the smallest refused |s| on those rows, to have cleared.
#[derive(Clone, Copy, Default)]
pub struct MirrorTrend {
    /// The last rung refused by its mirror: its offset and signed error.
    last: Option<(f64, f64)>,
    pub closed: bool,
}

impl MirrorTrend {
    /// Records a rung refused by its mirror at `offset` with signed error
    /// `error`. When it and the direction's last refused rung close the
    /// direction, and `close` allows it, the direction is closed and the
    /// earlier rung's (offset, signed error) returned.
    pub fn refused(&mut self, offset: f64, error: f64, close: bool) -> Option<(f64, f64)> {
        let earlier = self
            .last
            .filter(|&(_, earlier)| earlier.signum() == error.signum() && error.abs() >= earlier.abs())
            .filter(|_| close);
        self.last = Some((offset, error));
        self.closed |= earlier.is_some();
        earlier
    }

    /// A rung whose mirror clears: the direction's record starts again.
    pub fn cleared(&mut self) {
        self.last = None;
    }
}

/// Non-degenerate edges a solid's faces use exactly once.
/// BELOW-TOLERANCE REFUSAL (`BREP_BELOW_TOLERANCE_REFUSAL`, default on, `=0` off). Two
/// planar faces, one of each operand, that are nearly parallel and stand no
/// farther apart over their shared extent than [`BELOW_TOLERANCE_BAND`] but not
/// exactly on one another are a configuration the kernel cannot resolve: its
/// weld floor is 1e-5 and its face arrangements resolve a few 1e-5, so the
/// slivers between such faces fall under the modelling tolerance. A cube
/// against a copy turned 1e-4 deg moves its corners 1.2e-5 to 2.5e-5: the exact
/// arrangement failed, and the perturbation ladder then answered a nudged pair
/// 8.5e-5 out in volume; at 1e-6 deg the face pairs read as coincident and the
/// difference came back empty. Consulted only where the exact lane failed or
/// returned an empty difference or common part, so nothing that answers today
/// changes; the refusal names the pair and the modelling tolerance instead of
/// shipping a nudged or empty solid.
pub fn below_tolerance_refusal_enabled() -> bool {
    crate::switch!("BREP_BELOW_TOLERANCE_REFUSAL", true)
}

/// `BREP_BELOW_TOLERANCE_SLIVERS` (default on; `=0` refuses or keeps the exact
/// lane's answer as before): where the below-tolerance
/// refusal would name a pair, an operand whose difference from the other is
/// all sliver ([`difference_all_sliver`]) is read as lying inside it, and the
/// boolean answers by that inclusion: `first - second` is empty when it is all
/// sliver; `first ∪ second` is the operand the other lies inside; `first ∩
/// second` is the operand lying inside the other. A box less a copy of itself
/// moved 5.4e-6 and turned 8e-7 rad holds 1.1e-4 against a bar of 1.6e-4 and
/// was refused (the exact lane had no boundary face left to write); a box
/// united with a copy whose top stands 1e-5 under its own and whose other
/// faces stand within 2e-7 of its own came back with open loops where the
/// 1e-5 strips of the side faces welded shut. A union or common part the
/// exact lane did write over such a pair takes the inclusion's answer too: the
/// common part of that box and copy read 8e-5 apart in the two orders, both
/// above the copy's own volume. Anything else still refuses by name, or keeps
/// the exact lane's answer.
pub fn below_tolerance_slivers_enabled() -> bool {
    crate::switch!("BREP_BELOW_TOLERANCE_SLIVERS", true)
}

/// The most faces an operand of [`below_tolerance_answer`] may have: the
/// polyhedral reads are quadratic in the faces.
const SLIVER_ANSWER_FACES: usize = 64;

fn face_count(solid: &BrepSolid) -> usize {
    solid.shells.iter().map(|shell| shell.faces.len()).sum()
}

/// The answer [`below_tolerance_slivers_enabled`] gives `operation`, if any.
fn below_tolerance_answer(first: &BrepSolid, second: &BrepSolid, operation: BooleanOperation) -> Option<BrepSolid> {
    if !below_tolerance_slivers_enabled() || [first, second].iter().any(|solid| face_count(solid) > SLIVER_ANSWER_FACES) {
        return None;
    }
    if ![first, second].iter().all(|solid| closed_single_shell(solid)) {
        return None;
    }
    let inside = |x: &BrepSolid, y: &BrepSolid| difference_all_sliver(x, y);
    if matches!(operation, BooleanOperation::Subtract) {
        return inside(first, second).then(empty_solid);
    }
    let (first_in, second_in) = (inside(first, second), inside(second, first));
    // Each inside the other: both answers hold at the tolerance; the union
    // takes the larger and the common part the smaller, whichever comes first.
    let first_larger = || match (body_volume(first), body_volume(second)) {
        (Some(a), Some(b)) => a >= b,
        _ => true,
    };
    let outer_is_first = match (first_in, second_in) {
        (false, false) => return None,
        (true, false) => false,
        (false, true) => true,
        (true, true) => first_larger(),
    };
    let (outer, inner) = if outer_is_first { (first, second) } else { (second, first) };
    Some(match operation {
        BooleanOperation::Union => outer.clone(),
        _ => inner.clone(),
    })
}

/// One shell of a solid, every proper edge used by exactly two coedges.
fn closed_single_shell(solid: &BrepSolid) -> bool {
    if !solid.kind.is_solid() || solid.shells.len() != 1 {
        return false;
    }
    let mut uses: HashMap<u64, usize> = HashMap::default();
    for coedge in solid.shells[0].faces.iter().flat_map(|face| &face.loops).flat_map(|lp| &lp.coedges) {
        *uses.entry(coedge.edge_id).or_default() += 1;
    }
    !uses.is_empty() && uses.values().all(|&count| count == 2) && solid.edges.iter().all(|edge| uses.contains_key(&edge.id))
}

/// Whether `first - second` is all sliver: `second` a convex polyhedron and
/// `first` a polyhedron (planar faces, straight edges) standing past no face
/// plane of `second` by more than [`BELOW_TOLERANCE_BAND`], and the part of
/// `first` beyond `second`'s face planes, summed plane by plane (an upper bound
/// on V(first - second), since `second` is the intersection of their inner
/// half-spaces), at most `first`'s area times the coincidence distance
/// (1e-6): spread over the boundary, the material is thinner than two faces
/// the kernel reads as one surface. Each plane's part is exact: `first`'s face
/// polygons clipped to the plane's outer side, their volume read about a
/// point of the plane, where the cut's own cap adds nothing.
fn difference_all_sliver(first: &BrepSolid, second: &BrepSolid) -> bool {
    let Some(bounds) = difference_bounds(first, second) else {
        return false;
    };
    let Ok(properties) = crate::solid_mass_properties(first) else {
        return false;
    };
    bounds.out <= BELOW_TOLERANCE_BAND
        && bounds.upper.is_finite()
        && bounds.upper <= crate::tolerance::COINCIDENCE_DISTANCE_FLOOR * properties.surface_area
}

/// Bounds on V(first - second) for a polyhedral `first` and a convex
/// polyhedral `second`, from `first`'s parts beyond each face plane of
/// `second` (exact, see [`difference_all_sliver`]): each part lies in the
/// difference, so the largest is a lower bound, and the difference is their
/// union, so their sum is an upper one.
struct DifferenceBounds {
    lower: f64,
    upper: f64,
    /// How far `first` stands past `second`'s face planes at most.
    out: f64,
}

fn difference_bounds(first: &BrepSolid, second: &BrepSolid) -> Option<DifferenceBounds> {
    let scale = crate::solid_tolerance::solid_scale(first).max(crate::solid_tolerance::solid_scale(second)).max(1.0);
    let (polygons_a, polygons_b) = (face_polygons(first, scale)?, face_polygons(second, scale)?);
    let vertices_b: Vec<Vec3> = second.vertices.iter().map(|vertex| vertex.point).collect();
    let vertices_a: Vec<Vec3> = first.vertices.iter().map(|vertex| vertex.point).collect();
    let flat = 1e-9 * scale;
    let mut bounds = DifferenceBounds { lower: 0.0, upper: 0.0, out: 0.0 };
    for polygon in &polygons_b {
        let (origin, normal) = polygon_plane(polygon)?;
        // Convex: every vertex of `second` on the inner side of its face plane.
        if vertices_b.iter().any(|vertex| vertex.sub(origin).dot(normal) > flat) {
            return None;
        }
        let out = vertices_a.iter().map(|vertex| vertex.sub(origin).dot(normal)).fold(0.0f64, f64::max);
        bounds.out = bounds.out.max(out);
        if out <= 0.0 {
            continue;
        }
        let part: f64 = polygons_a.iter().map(|face| clipped_cone_volume(face, origin, normal)).sum();
        bounds.lower = bounds.lower.max(part);
        bounds.upper += part;
    }
    Some(bounds)
}

/// `BREP_BELOW_TOLERANCE_RESCALE` (default off): a
/// boolean the below-tolerance refusal would name, over two closed convex
/// polyhedra, is run again on both operands scaled up 100 (then 1000) times, so that faces a
/// weld floor apart (1e-5) stand clear of it, and the answer scaled back is
/// kept when its volume and area are the exact ones of the two convex
/// polyhedra ([`convex_pair_measures`]) to a part in 1e9: a cube less a copy
/// turned 1e-4 degrees came back right in volume but 1e-3 over in area (a
/// sliver's face too many), and refuses. A box less a copy whose top stands 1e-5 under its own
/// leaves a slab 1e-5 thick, a valid body: refused, it came back exact.
pub fn below_tolerance_rescale_enabled() -> bool {
    crate::switch!("BREP_BELOW_TOLERANCE_RESCALE", false)
}

fn below_tolerance_rescaled(first: &BrepSolid, second: &BrepSolid, operation: BooleanOperation, options: &BooleanOptions) -> Option<BrepSolid> {
    if !below_tolerance_rescale_enabled()
        || options.tolerances.is_some()
        || [first, second].iter().any(|solid| face_count(solid) > SLIVER_ANSWER_FACES || !closed_single_shell(solid))
    {
        return None;
    }
    let (volume_exact, area_exact) = convex_pair_measures(first, second, operation)?;
    let volume_slack = 1e-9 * body_volume(first)?.max(body_volume(second)?);
    let area_slack = 1e-9 * area_exact.max(1.0);
    for factor in [100.0, 1000.0] {
        let scaled = |solid: &BrepSolid| crate::rescale::scale_operand(solid, factor, "boolean").ok();
        let (Some(a), Some(b)) = (scaled(first), scaled(second)) else {
            continue;
        };
        let mut scaled_options = options.clone();
        scaled_options.imprint.maximum_ssi_step = scaled_options.imprint.maximum_ssi_step.map(|step| step * factor);
        let Ok(outcome) = boolean_operation_with_diagnostics(&a, &b, operation, &scaled_options) else {
            continue;
        };
        let Ok(answer) = crate::rescale::scale_operand(&outcome.value, 1.0 / factor, "boolean") else {
            continue;
        };
        let Ok(properties) = crate::solid_mass_properties(&answer) else {
            continue;
        };
        if (properties.volume.abs() - volume_exact).abs() <= volume_slack
            && (properties.surface_area - area_exact).abs() <= area_slack
            && !answer.validate().iter().any(|issue| issue.severity == "error")
        {
            return Some(answer);
        }
    }
    None
}

/// The exact volume and area of `operation` on two convex polyhedra: each
/// face polygon clipped to the other's inner half-spaces is the part of it
/// inside the other, the common part is closed by those parts of both, and
/// the union and difference follow from the parts inside and the rest. `None`
/// unless both are convex polyhedra.
fn convex_pair_measures(first: &BrepSolid, second: &BrepSolid, operation: BooleanOperation) -> Option<(f64, f64)> {
    let scale = crate::solid_tolerance::solid_scale(first).max(crate::solid_tolerance::solid_scale(second)).max(1.0);
    let (polygons_a, polygons_b) = (face_polygons(first, scale)?, face_polygons(second, scale)?);
    let (planes_a, planes_b) = (convex_planes(&polygons_a, scale)?, convex_planes(&polygons_b, scale)?);
    let zero = Vec3::new(0.0, 0.0, 0.0);
    // (volume of the cones over the parts inside, area of the polygons, area of the parts inside)
    let read = |polygons: &[Vec<Vec3>], planes: &[(Vec3, Vec3)]| {
        let (mut cone, mut area, mut inside_area) = (0.0, 0.0, 0.0);
        for polygon in polygons {
            area += vector_area(polygon, zero).length();
            let mut inside = polygon.clone();
            for &(origin, normal) in planes {
                inside = clip_polygon(&inside, origin, normal.scale(-1.0));
                if inside.len() < 3 {
                    break;
                }
            }
            if inside.len() >= 3 {
                let vector = vector_area(&inside, zero);
                cone += inside[0].dot(vector) / 3.0;
                inside_area += vector.length();
            }
        }
        (cone, area, inside_area)
    };
    let (cone_a, area_a, inside_a) = read(&polygons_a, &planes_b);
    let (cone_b, area_b, inside_b) = read(&polygons_b, &planes_a);
    let (volume_a, volume_b) = (body_volume(first)?, body_volume(second)?);
    let common = cone_a + cone_b;
    Some(match operation {
        BooleanOperation::Intersect => (common, inside_a + inside_b),
        BooleanOperation::Union => (volume_a + volume_b - common, (area_a - inside_a) + (area_b - inside_b)),
        BooleanOperation::Subtract => (volume_a - common, (area_a - inside_a) + inside_b),
    })
}

/// A convex polyhedron's face planes (a point and the outward unit normal),
/// `None` when some vertex stands outside one of them.
fn convex_planes(polygons: &[Vec<Vec3>], scale: f64) -> Option<Vec<(Vec3, Vec3)>> {
    let planes = polygons.iter().map(|polygon| polygon_plane(polygon)).collect::<Option<Vec<_>>>()?;
    let flat = 1e-9 * scale;
    let convex = planes.iter().all(|&(origin, normal)| polygons.iter().flatten().all(|vertex| vertex.sub(origin).dot(normal) <= flat));
    convex.then_some(planes)
}

/// The part of `polygon` on the side of the plane through `origin` that
/// `normal` points to (Sutherland-Hodgman).
fn clip_polygon(polygon: &[Vec3], origin: Vec3, normal: Vec3) -> Vec<Vec3> {
    let side = |point: Vec3| point.sub(origin).dot(normal);
    let mut kept = Vec::with_capacity(polygon.len() + 2);
    for (index, &point) in polygon.iter().enumerate() {
        let next = polygon[(index + 1) % polygon.len()];
        let (here, there) = (side(point), side(next));
        if here >= 0.0 {
            kept.push(point);
        }
        if (here >= 0.0) != (there >= 0.0) {
            let t = here / (here - there);
            kept.push(point.add(next.sub(point).scale(t)));
        }
    }
    kept
}

/// Every loop of `solid` as a polygon of its vertices, ordered so the
/// polygons' vector areas point out of the solid; `None` unless every face is
/// a plane and every edge a straight segment.
fn face_polygons(solid: &BrepSolid, scale: f64) -> Option<Vec<Vec<Vec3>>> {
    let vertex = |id: u64| solid.vertices.iter().find(|vertex| vertex.id == id).map(|vertex| vertex.point);
    let mut polygons = Vec::new();
    for face in solid.shells.iter().flat_map(|shell| &shell.faces) {
        if !face.surface.is_affine().ok()? {
            return None;
        }
        for lp in &face.loops {
            let mut polygon = Vec::with_capacity(lp.coedges.len());
            for coedge in &lp.coedges {
                let edge = solid.edges.iter().find(|edge| edge.id == coedge.edge_id)?;
                let (start, end) = (vertex(edge.start_vertex_id)?, vertex(edge.end_vertex_id)?);
                let chord = end.sub(start);
                let length = chord.length();
                if edge.degenerate || length <= 1e-12 * scale {
                    return None;
                }
                for control in &edge.curve.control_points {
                    let offset = control.point().ok()?.sub(start);
                    if offset.cross(chord).length() / length > 1e-9 * scale {
                        return None;
                    }
                }
                polygon.push(if coedge.forward { start } else { end });
            }
            polygons.push(polygon);
        }
    }
    // Orient by the enclosed volume's sign.
    let volume: f64 = polygons.iter().map(|polygon| clipped_cone_volume(polygon, Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 0.0))).sum();
    if !(volume.abs() > 0.0) {
        return None;
    }
    if volume < 0.0 {
        for polygon in &mut polygons {
            polygon.reverse();
        }
    }
    Some(polygons)
}

/// A polygon's plane: a point of it and its unit outward normal.
fn polygon_plane(polygon: &[Vec3]) -> Option<(Vec3, Vec3)> {
    let origin = *polygon.first()?;
    Some((origin, vector_area(polygon, origin).normalized().ok()?))
}

fn vector_area(polygon: &[Vec3], origin: Vec3) -> Vec3 {
    let mut area = Vec3::new(0.0, 0.0, 0.0);
    for (index, point) in polygon.iter().enumerate() {
        let next = polygon[(index + 1) % polygon.len()];
        area = area.add(point.sub(origin).cross(next.sub(origin)));
    }
    area.scale(0.5)
}

/// The volume of the cone from `origin` over the part of `polygon` on the
/// outer side of the plane through `origin` with `normal` (all of it for a
/// zero normal). Summed over a closed polyhedron's polygons, it is the volume
/// of the polyhedron beyond that plane: the cut's cap lies in the plane and
/// adds nothing about a point of it.
fn clipped_cone_volume(polygon: &[Vec3], origin: Vec3, normal: Vec3) -> f64 {
    let clipped: Vec<Vec3> = if normal.length() == 0.0 { polygon.to_vec() } else { clip_polygon(polygon, origin, normal) };
    if clipped.len() < 3 {
        return 0.0;
    }
    clipped[0].sub(origin).dot(vector_area(&clipped, origin)) / 3.0
}

/// Ten weld floors: the separation under which two near-coincident planar faces
/// are read as below the modelling tolerance.
const BELOW_TOLERANCE_BAND: f64 = 10.0 * crate::tolerance::WELD_FLOOR;

/// UNRESOLVED-MERGE REFUSAL (`BREP_UNRESOLVED_MERGE_REFUSAL`, default off). A
/// union or common part that SUCCEEDS over two planar faces, one of each
/// operand, standing farther apart than the coincidence band (1e-6: closer
/// faces are one surface by the kernel's definition) but no farther than the
/// weld floor (1e-5) has welded the two faces into one: a unit cube united
/// with a copy raised 3e-6 came back as the cube, valid, 2e-6 out in volume,
/// where the difference of the same pair already refuses by this name. The
/// step between such faces is neither a coincidence nor resolvable, so the
/// answer is the refusal.
pub fn unresolved_merge_refusal_enabled() -> bool {
    crate::switch!("BREP_UNRESOLVED_MERGE_REFUSAL", false)
}

/// SLIVER SHELLS (`BREP_SLIVER_SHELL_DROP`, default on, `=0` off). A solid
/// result of several shells, one of which is closed and encloses no material
/// (its volume over its area, the thickness of the slab it would be, under
/// [`SLIVER_SHELL_THICKNESS`] of the body's size: round-off), carries a sheet the arrangement could not
/// resolve, left closed on itself: a measure-zero set, not a void. A cylinder
/// of radius 5 with a coaxial one of radius 4.99999 between the same end
/// planes gave their common part the inner cylinder plus 64 closed sheets
/// over the 1e-5 rings at its ends, 6e-4 over in area and refused by STEP
/// export ("a void shell lies inside no outward shell"). Such shells are
/// dropped, with the edges and vertices only they used; the point set is
/// unchanged. Sheet and other non-solid bodies, open shells, and a body whose
/// every shell would go are left as they are.
pub fn sliver_shell_drop_enabled() -> bool {
    crate::switch!("BREP_SLIVER_SHELL_DROP", true)
}

/// The thickness, as a fraction of the body's size, under which a closed shell
/// encloses nothing: round-off. The sheets read 0 and 7e-17 thick; a real
/// slab 1e-7 thick beside a unit cube is material and stays.
const SLIVER_SHELL_THICKNESS: f64 = 1e-12;

/// `BREP_SLIVER_SHELL_FAST` (on by default; `=0` reads every shell in full):
/// a shell is kept without its full
/// mass properties when its volume, read for its sign against the floor
/// times an upper bound on its area ([`face_area_upper_bound`]), proves it
/// thicker than the floor (only where every face's trim lies within its
/// carrier's knot domain). The reading's verdict is the exact check's, and
/// an area above the shell's makes the bar higher than the exact one, so a
/// shell kept this way is one the exact reading keeps. Everything else (a void,
/// whose volume is negative, a real sliver, a face without a bound) is read
/// in full as before. The reading reuses the face terms the assembly's sign
/// check retained: on a screw wall crossed by a box, the check of four shells
/// fell from about 300 ms to a few. `BREP_SLIVER_SHELL_FAST_CHECK` reads both
/// and counts the shells whose verdicts differ
/// ([`sliver_shell_fast_check_take`]), keeping the full reading's.
pub fn sliver_shell_fast_on() -> bool {
    crate::switch!("BREP_SLIVER_SHELL_FAST", true)
}

fn sliver_shell_fast_check_on() -> bool {
    crate::switch!("BREP_SLIVER_SHELL_FAST_CHECK", false)
}

thread_local! {
    static SLIVER_FAST_CHECK: std::cell::Cell<(u64, u64, u64)> = const { std::cell::Cell::new((0, 0, 0)) };
}

/// (shells read both ways, verdicts that differed, shells the fast reading
/// kept) on this thread since the last call.
pub fn sliver_shell_fast_check_take() -> (u64, u64, u64) {
    SLIVER_FAST_CHECK.with(|cell| cell.replace((0, 0, 0)))
}

fn sliver_fast_record(compared: u64, differed: u64, kept: u64) {
    SLIVER_FAST_CHECK.with(|cell| {
        let (a, b, c) = cell.get();
        cell.set((a + compared, b + differed, c + kept));
    });
}

/// An upper bound on the area of `surface` over its whole parameter domain
/// (so on any face trimmed from it). `|S_u × S_v| ≤ |S_u| |S_v|`, and with
/// `S = P / w` (homogeneous `P`, weight `w`) `|S_u| ≤ (|P_u| w_max + |P|
/// |w_u|) / w_min²`. Each factor is bounded by the convex hull of its control
/// net: `|P|` and the weights by the control points, `P_u` and `w_u` by the
/// hodograph's points `p (Q_{i+1} − Q_i) / (u_{i+p+1} − u_{i+1})`. `None`
/// for a weight that is not positive or a bound that is not finite.
fn face_area_upper_bound(surface: &crate::NurbsSurface) -> Option<f64> {
    let net = &surface.control_points;
    let rows = net.len();
    let columns = net.first()?.len();
    if rows < 2 || columns < 2 || net.iter().any(|row| row.len() != columns) {
        return None;
    }
    let (p, q) = (surface.degree_u, surface.degree_v);
    let (us, vs) = (&surface.knots_u, &surface.knots_v);
    if us.len() != rows + p + 1 || vs.len() != columns + q + 1 {
        return None;
    }
    let mut w_min = f64::INFINITY;
    let mut w_max = 0.0f64;
    let mut point_max = 0.0f64;
    for c in net.iter().flatten() {
        if !(c.w > 0.0) {
            return None;
        }
        w_min = w_min.min(c.w);
        w_max = w_max.max(c.w);
        point_max = point_max.max((c.x * c.x + c.y * c.y + c.z * c.z).sqrt());
    }
    // (max |P_u|, max |w_u|) over the hodograph in one direction.
    let hodograph = |along_u: bool| -> Option<(f64, f64)> {
        let (count, degree, knots) = if along_u { (rows, p, us) } else { (columns, q, vs) };
        let (mut dp, mut dw) = (0.0f64, 0.0f64);
        for i in 0..count - 1 {
            let span = knots[i + degree + 1] - knots[i + 1];
            if !(span > 0.0) {
                continue;
            }
            let scale = degree as f64 / span;
            let others = if along_u { columns } else { rows };
            for j in 0..others {
                let (a, b) = if along_u { (net[i][j], net[i + 1][j]) } else { (net[j][i], net[j][i + 1]) };
                let (x, y, z) = (b.x - a.x, b.y - a.y, b.z - a.z);
                dp = dp.max(scale * (x * x + y * y + z * z).sqrt());
                dw = dw.max(scale * (b.w - a.w).abs());
            }
        }
        Some((dp, dw))
    };
    let (pu, wu) = hodograph(true)?;
    let (pv, wv) = hodograph(false)?;
    let su = (pu * w_max + point_max * wu) / (w_min * w_min);
    let sv = (pv * w_max + point_max * wv) / (w_min * w_min);
    let extent_u = us[us.len() - 1] - us[0];
    let extent_v = vs[vs.len() - 1] - vs[0];
    let bound = su * sv * extent_u * extent_v;
    bound.is_finite().then_some(bound)
}

/// [`face_area_upper_bound`] of `face`'s carrier, when its trim lies within
/// the carrier's knot domain: every pcurve control point (whose convex hull
/// holds the pcurve) inside it, to `1e-12` of the domain's own extent (a circle's
/// corner points land a rounding outside). A trim reaching further is
/// measured on the carrier's extension, which the control net does not
/// bound: `None`. The bound is doubled to cover that rounding's sliver of
/// extension (the carrier's own polynomial continued).
pub fn trimmed_face_area_upper_bound(face: &crate::topology::FaceRecord) -> Option<f64> {
    let s = &face.surface;
    let (u0, u1) = (*s.knots_u.first()?, *s.knots_u.last()?);
    let (v0, v1) = (*s.knots_v.first()?, *s.knots_v.last()?);
    // (relative to the domain alone: a tiny chart's allowance stays tiny)
    let (du, dv) = (1e-12 * (u1 - u0).abs(), 1e-12 * (v1 - v0).abs());
    if !(du > 0.0 && dv > 0.0) {
        return None;
    }
    for coedge in face.loops.iter().flat_map(|lp| &lp.coedges) {
        for c in &coedge.pcurve.control_points {
            if !(c.w > 0.0) {
                return None;
            }
            let (u, v) = (c.x / c.w, c.y / c.w);
            if !(u >= u0 - du && u <= u1 + du && v >= v0 - dv && v <= v1 + dv) {
                return None;
            }
        }
    }
    face_area_upper_bound(s).map(|bound| 2.0 * bound)
}

/// Drops the sliver shells of `solid` (see [`sliver_shell_drop_enabled`]) and
/// returns how many went.
fn drop_sliver_shells(solid: &mut BrepSolid) -> usize {
    if !sliver_shell_drop_enabled() || !solid.kind.is_solid() || solid.shells.len() < 2 {
        return 0;
    }
    let floor = SLIVER_SHELL_THICKNESS * crate::solid_tolerance::solid_scale(solid).max(1.0);
    let degenerate: std::collections::HashSet<u64> = solid.edges.iter().filter(|edge| edge.degenerate).map(|edge| edge.id).collect();
    let mut drop = vec![false; solid.shells.len()];
    for (index, shell) in solid.shells.iter().enumerate() {
        if shell.faces.is_empty() {
            continue;
        }
        // Closed: every proper edge the shell uses, used exactly twice by it.
        let mut uses: HashMap<u64, usize> = HashMap::default();
        for coedge in shell.faces.iter().flat_map(|face| &face.loops).flat_map(|lp| &lp.coedges) {
            if !degenerate.contains(&coedge.edge_id) {
                *uses.entry(coedge.edge_id).or_default() += 1;
            }
        }
        if uses.is_empty() || uses.values().any(|&count| count != 2) {
            continue;
        }
        let alone = BrepSolid { shells: vec![shell.clone()], mass_properties_cache: Default::default(), ..solid.clone() };
        let (fast, check) = (sliver_shell_fast_on(), sliver_shell_fast_check_on());
        let kept_fast = (fast || check)
            && shell
                .faces
                .iter()
                .map(trimmed_face_area_upper_bound)
                .sum::<Option<f64>>()
                .map(|area| floor * area)
                .filter(|bar| bar.is_finite() && *bar > 0.0)
                .and_then(|bar| crate::solid_signed_volume_for_sign(&alone, bar).ok().map(|volume| volume > bar))
                .unwrap_or(false);
        if kept_fast && !check {
            sliver_fast_record(0, 0, 1);
            continue;
        }
        let Ok(mass) = crate::solid_mass_properties(&alone) else {
            continue;
        };
        let thickness = mass.volume.abs() / mass.surface_area.max(f64::MIN_POSITIVE);
        if env_read!("BREP_DEBUG_SLIVER_SHELL").is_some() {
            eprintln!(
                "sliver-shell: shell {index} of {}: volume {:.6e} area {:.6e} thickness {thickness:.3e}",
                solid.shells.len(),
                mass.volume,
                mass.surface_area
            );
        }
        drop[index] = thickness < floor;
        if check {
            sliver_fast_record(1, u64::from(kept_fast && drop[index]), u64::from(kept_fast));
        }
    }
    let dropped = drop.iter().filter(|&&d| d).count();
    if dropped == 0 || dropped == solid.shells.len() {
        return 0;
    }
    let mut index = 0;
    solid.shells.retain(|_| {
        let keep = !drop[index];
        index += 1;
        keep
    });
    let used_edges: std::collections::HashSet<u64> = solid
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .flat_map(|face| &face.loops)
        .flat_map(|lp| &lp.coedges)
        .map(|coedge| coedge.edge_id)
        .collect();
    solid.edges.retain(|edge| used_edges.contains(&edge.id));
    let used_vertices: std::collections::HashSet<u64> =
        solid.edges.iter().flat_map(|edge| [edge.start_vertex_id, edge.end_vertex_id]).collect();
    solid.vertices.retain(|vertex| used_vertices.contains(&vertex.id));
    solid.mass_properties_cache = Default::default();
    // A dropped shell may have carried handles: the genus is read again.
    refresh_derived_genus(solid);
    dropped
}

fn unresolved_merge_refusal(first: &BrepSolid, second: &BrepSolid) -> Option<KernelRefusal> {
    let (lower, upper) = (crate::tolerance::COINCIDENCE_DISTANCE_FLOOR, crate::tolerance::WELD_FLOOR);
    let (face_a, face_b, separation) = planar_faces_apart_within(first, second, lower, upper)?;
    Some(KernelRefusal::unsupported(
        KernelStage::Intersect,
        "below_modelling_tolerance",
        format!(
            "boolean: face {face_a} of the first operand and face {face_b} of the second are planes nearly on one \
             another, {separation:.3e} apart at most over their shared extent: farther than a coincidence \
             ({lower:.0e}) and closer than the kernel resolves ({upper:.0e}), so the step between them would be \
             welded away"
        ),
    ))
}

fn below_tolerance_refusal(first: &BrepSolid, second: &BrepSolid, error: Option<&KernelRefusal>) -> Option<KernelRefusal> {
    let (face_a, face_b, separation) = near_coincident_planar_faces(first, second)?;
    let message = format!(
        "boolean: face {face_a} of the first operand and face {face_b} of the second are planes nearly on one \
         another, {separation:.3e} apart at most over their shared extent: the slivers between them are below the \
         modelling tolerance ({BELOW_TOLERANCE_BAND:.0e}), so the kernel cannot resolve them{}",
        error.map(|error| format!(" (the exact arrangement said: {})", error.message.chars().take(160).collect::<String>())).unwrap_or_default()
    );
    Some(KernelRefusal::unsupported(KernelStage::Intersect, "below_modelling_tolerance", message))
}

/// The first pair of planar faces (one per operand) whose planes are nearly
/// parallel and whose boundaries stand within [`BELOW_TOLERANCE_BAND`] of the
/// other's plane, with overlapping extents, but which are not one plane.
fn near_coincident_planar_faces(first: &BrepSolid, second: &BrepSolid) -> Option<(u64, u64, f64)> {
    let scale = crate::solid_tolerance::solid_scale(first).max(crate::solid_tolerance::solid_scale(second)).max(1.0);
    planar_faces_apart_within(first, second, 1e-9 * scale, BELOW_TOLERANCE_BAND)
}

/// The first pair of nearly parallel planar faces (one per operand) with
/// overlapping extents whose largest separation is above `lower` and at most
/// `upper`.
fn planar_faces_apart_within(first: &BrepSolid, second: &BrepSolid, lower: f64, upper: f64) -> Option<(u64, u64, f64)> {
    struct Planar {
        id: u64,
        origin: Vec3,
        normal: Vec3,
        corners: Vec<Vec3>,
        bounds: crate::spatial::Aabb,
    }
    let read = |solid: &BrepSolid| -> Vec<Planar> {
        let at = |id: u64| solid.vertices.iter().find(|vertex| vertex.id == id).map(|vertex| vertex.point);
        solid
            .shells
            .iter()
            .flat_map(|shell| &shell.faces)
            .filter_map(|face| {
                if !face.surface.is_affine().ok()? {
                    return None;
                }
                let origin = face.surface.control_points[0][0].point().ok()?;
                let a = face.surface.control_points[1][0].point().ok()?.sub(origin);
                let b = face.surface.control_points[0][1].point().ok()?.sub(origin);
                let normal = a.cross(b).normalized().ok()?;
                let corners: Vec<Vec3> = face
                    .loops
                    .iter()
                    .flat_map(|lp| &lp.coedges)
                    .filter_map(|coedge| solid.edges.iter().find(|edge| edge.id == coedge.edge_id))
                    .flat_map(|edge| [at(edge.start_vertex_id), at(edge.end_vertex_id)])
                    .flatten()
                    .collect();
                if corners.is_empty() {
                    return None;
                }
                let bounds = crate::spatial::Aabb::from_points(corners.clone());
                Some(Planar { id: face.id, origin, normal, corners, bounds })
            })
            .collect()
    };
    let (faces_a, faces_b) = (read(first), read(second));
    for a in &faces_a {
        for b in &faces_b {
            if a.normal.cross(b.normal).length() > 1e-3 {
                continue;
            }
            if !a.bounds.intersects(b.bounds, upper) {
                continue;
            }
            let separation = a
                .corners
                .iter()
                .map(|corner| corner.sub(b.origin).dot(b.normal).abs())
                .chain(b.corners.iter().map(|corner| corner.sub(a.origin).dot(a.normal).abs()))
                .fold(0.0f64, f64::max);
            if separation > lower && separation <= upper {
                return Some((a.id, b.id, separation));
            }
        }
    }
    None
}

pub fn one_use_edges(solid: &BrepSolid) -> usize {
    let mut uses: HashMap<u64, usize> = HashMap::default();
    for coedge in solid
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .flat_map(|face| &face.loops)
        .flat_map(|loop_record| &loop_record.coedges)
    {
        *uses.entry(coedge.edge_id).or_default() += 1;
    }
    solid
        .edges
        .iter()
        .filter(|edge| !edge.degenerate && uses.get(&edge.id) == Some(&1))
        .count()
}

/// A body's enclosed volume, zero for an empty one.
fn body_volume(solid: &BrepSolid) -> Option<f64> {
    if solid.shells.is_empty() {
        return Some(0.0);
    }
    solid_signed_volume(solid).ok().map(f64::abs)
}

/// What the exact lane's other operations say `operation` on these operands
/// must enclose.
struct InclusionExclusion {
    /// The rescued operation's volume in terms of V(first), V(second) and
    /// V(first ∩ second).
    formula: &'static str,
    /// The exact-lane boolean that supplied V(first ∩ second).
    reading: &'static str,
    /// That boolean's enclosed volume.
    read: f64,
    /// The volume `formula` gives.
    expected: f64,
}

/// The volume `operation` on these operands must have, by inclusion–exclusion.
/// Any one exact-lane boolean of the same two operands fixes V(first ∩ second),
/// and with the operands' own volumes that fixes all four operations. The
/// readings are tried intersection, union, then difference, each in both
/// operand orders: the exact lane is not order-symmetric (the helmet's cutter
/// ∩ helmet refuses where helmet ∩ cutter clears). The rescued operation itself,
/// in its own order, is skipped, since it is the one that refused, unless
/// `own_first` asks for it to be read first (a rung's mirror, where it is a
/// different pair of operands). `None` when no reading clears or an operand has
/// no volume.
fn inclusion_exclusion_volume(
    first: &BrepSolid,
    second: &BrepSolid,
    operation: BooleanOperation,
    options: &BooleanOptions,
    va: Option<f64>,
    vb: Option<f64>,
    own_first: bool,
) -> Option<InclusionExclusion> {
    use BooleanOperation::{Intersect, Subtract, Union};
    let (va, vb) = (va?, vb?);
    let own = match operation {
        Intersect => (Intersect, false, "first ∩ second"),
        Union => (Union, false, "first ∪ second"),
        Subtract => (Subtract, false, "first − second"),
    };
    let readings = [
        (Intersect, false, "first ∩ second"),
        (Intersect, true, "second ∩ first"),
        (Union, false, "first ∪ second"),
        (Union, true, "second ∪ first"),
        (Subtract, false, "first − second"),
        (Subtract, true, "second − first"),
    ];
    let own_reading = own_first.then_some(own);
    let leading = usize::from(own_first);
    for (index, (other, swapped, reading)) in own_reading.into_iter().chain(readings).enumerate() {
        if index >= leading && !swapped && std::mem::discriminant(&other) == std::mem::discriminant(&operation) {
            continue;
        }
        let (x, y) = if swapped { (second, first) } else { (first, second) };
        let Some(read) = boolean_operation_with_diagnostics(x, y, other, options)
            .ok()
            .and_then(|outcome| body_volume(&outcome.value))
        else {
            continue;
        };
        let intersection = match (other, swapped) {
            (Intersect, _) => read,
            (Union, _) => va + vb - read,
            (Subtract, false) => va - read,
            (Subtract, true) => vb - read,
        };
        let (formula, expected) = match operation {
            Intersect => ("V(first ∩ second)", intersection),
            Union => ("V(first) + V(second) − V(first ∩ second)", va + vb - intersection),
            Subtract => ("V(first) − V(first ∩ second)", va - intersection),
        };
        return Some(InclusionExclusion { formula, reading, read, expected });
    }
    None
}

/// The worst planar face of either operand whose STRAIGHT boundary sits
/// farther off its plane than the assembly weld radius:
/// `(operand index, face id, gap, weld)`. Mirrors the STEP importer's planar
/// split predicate (`step_import::builder::planar_split`) on a finished
/// solid: a plane, a single loop of straight non-degenerate edges, and a
/// boundary vertex beyond the weld. `None` means no such face, so a failure
/// may still be a genuine degeneracy worth a perturbation rung. Only
/// consulted on the failure path.
fn planar_operand_off_its_plane(
    first: &BrepSolid,
    second: &BrepSolid,
    options: &BooleanOptions,
) -> Option<(u8, u64, f64, f64)> {
    let policy = options
        .tolerances
        .clone()
        .unwrap_or_else(|| KernelTolerances::for_pair(first, second, options.tolerance));
    let weld = crate::tolerance::assembler_weld(policy.model);
    let mut worst: Option<(u8, u64, f64)> = None;
    for (operand, solid) in [(0u8, first), (1u8, second)] {
        let vertex_point = |id: u64| solid.vertices.iter().find(|v| v.id == id).map(|v| v.point);
        for face in solid.shells.iter().flat_map(|shell| &shell.faces) {
            let Some(crate::AnalyticSurface::Plane {
                origin,
                u_dir,
                v_dir,
                ..
            }) = face.surface.analytic()
            else {
                continue;
            };
            let [loop_record] = face.loops.as_slice() else {
                continue;
            };
            let Ok(normal) = u_dir.cross(*v_dir).normalized() else {
                continue;
            };
            let mut off_plane = 0.0f64;
            let mut straight = true;
            for coedge in &loop_record.coedges {
                let Some(edge) = solid.edges.iter().find(|edge| edge.id == coedge.edge_id) else {
                    straight = false;
                    break;
                };
                let (Some(start), Some(end)) =
                    (vertex_point(edge.start_vertex_id), vertex_point(edge.end_vertex_id))
                else {
                    straight = false;
                    break;
                };
                let chord = end.sub(start);
                let length = chord.length();
                if edge.degenerate || length <= weld {
                    straight = false;
                    break;
                }
                let direction = chord.scale(1.0 / length);
                for fraction in [0.25, 0.5, 0.75] {
                    let Ok(point) = edge.curve.evaluate(edge.t0 + (edge.t1 - edge.t0) * fraction) else {
                        straight = false;
                        break;
                    };
                    let delta = point.sub(start);
                    let along = delta.dot(direction);
                    if delta.sub(direction.scale(along)).length() > weld
                        || along < -weld
                        || along > length + weld
                    {
                        straight = false;
                        break;
                    }
                }
                if !straight {
                    break;
                }
                off_plane = off_plane.max(start.sub(*origin).dot(normal).abs());
            }
            if straight && off_plane > weld && worst.is_none_or(|(_, _, gap)| off_plane > gap) {
                worst = Some((operand, face.id, off_plane));
            }
        }
    }
    worst.map(|(operand, face_id, gap)| (operand, face_id, gap, weld))
}

/// CSG volume inequality bounds (no extra boolean needed) with 1% relative slack.
/// `va`, `vb`, `vr` are absolute volumes of operand A, operand B, and the result.
fn volume_within_csg_bounds(operation: BooleanOperation, va: f64, vb: f64, vr: f64) -> bool {
    let slack = 1e-2 * (va + vb).max(1.0);
    match operation {
        // max(A,B) ≤ A∪B ≤ A+B
        BooleanOperation::Union => vr >= va.max(vb) - slack && vr <= va + vb + slack,
        // A−B ≤ A (and ≥ 0)
        BooleanOperation::Subtract => vr <= va + slack && vr >= -slack,
        // A∩B ≤ min(A,B)
        BooleanOperation::Intersect => vr <= va.min(vb) + slack && vr >= -slack,
    }
}

// ---------------------------------------------------------------------------
// N-ary booleans (Golovanov T4.4)
//
// Union / intersect / subtract of N solids in ONE imprint+fragment pass.  This
// is a NEW ADDITIVE entry point: the binary `boolean_operation` above is left
// byte-identical.  The pipeline mirrors the binary one but generalizes the two
// operand-specific stages to N operands:
//
//   1. Imprint EVERY unordered pair (i, j) so every mutual intersection edge is
//      present on both operands' faces, and accumulate all pairs into ONE global
//      imprint (operand indices remapped to 0..N, piece/vertex ids made globally
//      unique, per-face piece lists and per-edge split parameters merged).
//   2. Split + fragment each operand by that single global imprint, so each
//      operand's faces are cut by ALL of its intersections at once.
//   3. Select fragments in one pass: the per-fragment keep decision generalizes
//      from "vs one other solid" to "vs the SET of other solids" (see
//      `nary_keep`).  Because every pair is imprinted, a fragment's interior lies
//      wholly inside or wholly outside each other operand, so a single interior
//      test point per (fragment, other-solid) is an exact membership verdict.
//   4. Assemble with the same machinery (`assemble_fragments`), then merge and
//      validate exactly as the binary path does.
// ---------------------------------------------------------------------------

/// Reverse a fragment's orientation (surface sense + every coedge), the same
/// transform the binary Subtract path applies to the second operand's kept
/// fragments.  Extracted so the n-ary selector can reuse it verbatim.
fn reverse_fragment(fragment: &mut FaceFragmentRecord) -> Result<(), KernelRefusal> {
    fragment.same_sense = !fragment.same_sense;
    for loop_record in &mut fragment.loops {
        loop_record.coedges.reverse();
        for coedge in &mut loop_record.coedges {
            coedge.forward = !coedge.forward;
            coedge.pcurve = coedge.pcurve.reversed().or_refuse(KernelStage::Validate, "reversed")?;
        }
    }
    Ok(())
}

/// Imprint every unordered pair of operands and fold the pairwise
/// `ImprintResultRecord`s into ONE global imprint whose operand indices are the
/// operands' global positions (0..N).  Piece and vertex ids are re-based to be
/// globally unique; `by_face` piece lists and `edge_splits` parameter lists are
/// MERGED per key so `fragment_face` (which takes the first `by_face` match) and
/// `apply_edge_splits` (which collects one entry per edge) see every cut.
fn build_nary_imprint(
    operands: &[BrepSolid],
    options: &ImprintOptions,
) -> Result<ImprintResultRecord, KernelRefusal> {
    let mut section_evidence = false;
    let mut vertices: Vec<ImprintVertex> = Vec::new();
    let mut pieces: Vec<ImprintPieceRecord> = Vec::new();
    let mut by_face: HashMap<(u8, u64), Vec<u64>> = HashMap::default();
    let mut edge_splits: HashMap<(u8, u64), Vec<f64>> = HashMap::default();
    let mut next_vertex_id: u64 = 1;
    let mut next_piece_id: u64 = 1;

    for i in 0..operands.len() {
        for j in (i + 1)..operands.len() {
            let pair = build_imprints(&operands[i], &operands[j], options)?;
            let remap = |operand: u8| -> u8 {
                if operand == 0 {
                    i as u8
                } else {
                    j as u8
                }
            };

            section_evidence = section_evidence || pair.section_evidence;
            let mut vertex_map: HashMap<u64, u64> = HashMap::default();
            for vertex in &pair.vertices {
                let global = next_vertex_id;
                next_vertex_id += 1;
                vertex_map.insert(vertex.id, global);
                vertices.push(ImprintVertex {
                    id: global,
                    point: vertex.point,
                });
            }

            let mut piece_map: HashMap<u64, u64> = HashMap::default();
            for piece in &pair.pieces {
                let global = next_piece_id;
                next_piece_id += 1;
                piece_map.insert(piece.id, global);
                let mut mapped = piece.clone();
                mapped.id = global;
                mapped.start_vertex_id = *vertex_map
                    .get(&piece.start_vertex_id)
                    .ok_or_else(|| KernelRefusal::internal(KernelStage::Intersect, "nary.imprint", "n-ary imprint: piece references unknown vertex"))?;
                mapped.end_vertex_id = *vertex_map
                    .get(&piece.end_vertex_id)
                    .ok_or_else(|| KernelRefusal::internal(KernelStage::Intersect, "nary.imprint", "n-ary imprint: piece references unknown vertex"))?;
                for pcurve in &mut mapped.pcurves {
                    pcurve.operand = remap(pcurve.operand);
                }
                mapped.support_faces = [
                    FaceKey {
                        operand: remap(piece.support_faces[0].operand),
                        face_id: piece.support_faces[0].face_id,
                    },
                    FaceKey {
                        operand: remap(piece.support_faces[1].operand),
                        face_id: piece.support_faces[1].face_id,
                    },
                ];
                pieces.push(mapped);
            }

            for entry in &pair.by_face {
                let list = by_face
                    .entry((remap(entry.operand), entry.face_id))
                    .or_default();
                for piece_id in &entry.piece_ids {
                    let global = *piece_map.get(piece_id).ok_or_else(|| {
                        KernelRefusal::internal(KernelStage::Intersect, "nary.imprint", "n-ary imprint: by_face references unknown piece")
                    })?;
                    list.push(global);
                }
            }

            for split in &pair.edge_splits {
                edge_splits
                    .entry((remap(split.operand), split.edge_id))
                    .or_default()
                    .extend(split.parameters.iter().copied());
            }
        }
    }

    Ok(ImprintResultRecord {
        faces_in_unproved_pairs: None,
        vertices,
        pieces,
        tangent_nodes: Vec::new(),
        barrier_edges: Vec::new(),
        section_evidence,
        cosurface_pairs: Vec::new(),
        operand_sliver_regions: Vec::new(),
        tangent_sections: Vec::new(),
        by_face: by_face
            .into_iter()
            .map(|((operand, face_id), piece_ids)| FaceImprints {
                operand,
                face_id,
                piece_ids,
            })
            .collect(),
        edge_splits: edge_splits
            .into_iter()
            .map(|((operand, edge_id), parameters)| EdgeSplitRecord {
                operand,
                edge_id,
                parameters,
            })
            .collect(),
    })
}

/// N-ary union / intersect / subtract of `operands` in ONE imprint+fragment
/// pass (Golovanov T4.4).  Additive: does not touch the binary path.
///
/// - `Union`   : the boundary of `∪ operands`.
/// - `Intersect`: the boundary of `∩ operands`.
/// - `Subtract`: `operands[0]` minus the union of `operands[1..]`.
///
/// `N = 1` returns the sole operand; `N = 2` runs the same general pipeline and
/// reproduces the binary result (same volume, valid).  Returns a clear `Err`
/// when the operands cannot assemble into a closed, valid solid.
pub fn boolean_operation_nary(
    operands: &[BrepSolid],
    operation: BooleanOperation,
) -> Result<BrepSolid, KernelRefusal> {
    boolean_operation_nary_with_options(operands, operation, &BooleanOptions::default())
}

/// [`boolean_operation_nary`] under the caller's [`BooleanOptions`]: the
/// tolerance, the coplanar-face merge and its `keep_unmerged_name_substrs`
/// list are honoured exactly as the binary path honours them, so a caller that
/// falls back from a sequence of binary unions to one arrangement (the sheet
/// metal walk with adjacent bends) keeps the same face partition contract.
pub fn boolean_operation_nary_with_options(
    operands: &[BrepSolid],
    operation: BooleanOperation,
    options: &BooleanOptions,
) -> Result<BrepSolid, KernelRefusal> {
    // More operands than one arrangement takes (255): folded a chunk at a
    // time, the result so far the next chunk's first operand. Union and
    // intersection are associative, and a difference takes the first operand
    // less all the rest, so the answer is the one arrangement's would be.
    let most = nary_chunk();
    if operands.len() > most && nary_chunks_on() {
        let mut whole = boolean_operation_nary_with_options(&operands[..most], operation, options)?;
        for chunk in operands[most..].chunks(most - 1) {
            // A last chunk of one operand is the binary boolean of the two
            // (as every two-operand call is answered by the pairwise path).
            if let [only] = chunk {
                whole = boolean_operation(&whole, only, operation, options)?;
                continue;
            }
            let mut next = Vec::with_capacity(chunk.len() + 1);
            next.push(whole);
            next.extend_from_slice(chunk);
            whole = boolean_operation_nary_with_options(&next, operation, options)?;
        }
        return Ok(whole);
    }
    let arranged = boolean_operation_nary_arranged(operands, operation, options);
    match arranged {
        Err(error) if nary_sequential_fallback_enabled() && operands.len() > 2 && error.message.contains("boolean assembly") => {
            // The binary path, operand by operand: each step is the pairwise
            // boolean every other caller makes.
            let mut whole = operands[0].clone();
            for operand in &operands[1..] {
                whole = match boolean_operation(&whole, operand, operation, options) {
                    Ok(solid) => solid,
                    Err(_) => return Err(error),
                };
            }
            Ok(whole)
        }
        other => other,
    }
}

/// `BREP_NARY_SEQUENTIAL_FALLBACK` (default on; `=0` refuses as before): an n-ary boolean whose single
/// arrangement fails to assemble is answered by the binary boolean applied
/// operand by operand, and refuses as before only if that fails too. A disc
/// less a counterbore, a hole and a countersink whose tops lie on the
/// counterbore's floor (three faces on one plane, the countersink's rim
/// crossing the counterbore wall's foot) left an open loop as one arrangement
/// and builds cut by cut.
/// The operands one n-ary arrangement takes.
const NARY_MOST: usize = 255;

/// The operands a fold takes a chunk at a time: `NARY_MOST` (a test may set
/// fewer on its thread).
fn nary_chunk() -> usize {
    NARY_MOST
}


/// `BREP_NARY_CHUNKS` (on unless =0): an n-ary boolean of more operands than
/// one arrangement takes is folded a chunk at a time instead of refused.
fn nary_chunks_on() -> bool {
    crate::switch!("BREP_NARY_CHUNKS", true)
}

fn nary_sequential_fallback_enabled() -> bool {
    crate::switch!("BREP_NARY_SEQUENTIAL_FALLBACK", true)
}

fn boolean_operation_nary_arranged(
    operands: &[BrepSolid],
    operation: BooleanOperation,
    options: &BooleanOptions,
) -> Result<BrepSolid, KernelRefusal> {
    if operands.len() > 1 {
        if let Some(scaled) = WorkingScale::of(&operands.iter().collect::<Vec<_>>(), options)? {
            return scaled.run(
                || boolean_operation_nary_with_options(operands, operation, options),
                || boolean_operation_nary_with_options(&scaled.solids, operation, &scaled.options)
                    .map_err(|error| scaled.refusal(error))
                    .and_then(|solid| scaled.back(&solid)),
            );
        }
    }
    if operands.is_empty() {
        return Err(KernelRefusal::input(KernelStage::Collect, "operands", "boolean_operation_nary: no operands provided"));
    }
    if operands.len() == 1 {
        return Ok(operands[0].clone());
    }
    if operands.len() > nary_chunk() {
        return Err(KernelRefusal::input(KernelStage::Collect, "operands", "boolean_operation_nary: at most 255 operands supported"));
    }
    // Each whole ball turned onto the one line the others cut it about
    // (`ball_align`); the turned set runs once more and is then left as it is.
    if let Some(aligned) = ball_align::align_balls_nary(operands, operation, options.tolerance) {
        return boolean_operation_nary_with_options(&aligned, operation, options);
    }
    let debug = std::env::var("BREP_DEBUG_BOOL").is_ok();

    // One tolerance policy for the whole set: the largest-scale operand's policy
    // dominates (every derived band is monotonic in scale), matching how the
    // binary path takes `for_pair` = max of the two scales.
    let mut policy = KernelTolerances::for_solid(&operands[0], options.tolerance);
    for operand in &operands[1..] {
        let candidate = KernelTolerances::for_solid(operand, options.tolerance);
        if candidate.sew_search > policy.sew_search {
            policy = candidate;
        }
    }
    policy.check().or_input(KernelStage::Collect, "tolerances")?;
    let tolerance = policy.model;

    // Per-operand fuse-first healing, exactly as the binary path (a no-op on
    // clean input, so clean operands are untouched).
    let mut healed = operands.to_vec();
    for operand in &mut healed {
        crate::heal::heal_operands(operand, &policy)?;
        // Then refit any inherited trim off the pcurve construction contract
        // whose edge lies on every carrier (a no-op without one).
        crate::heal::refit_inherited_trims_reused(operand, &policy)?;
        // Same operand normalization as the binary path: seam edges for
        // seamless full-period band faces.
        normalize_operand_band_seams(operand)?;
    }

    // 1. Imprint every pair into one global imprint.
    let mut imprint_options = options.imprint.clone();
    imprint_options.tolerance = tolerance;
    let imprint = build_nary_imprint(&healed, &imprint_options)?;

    // 2. Split every operand's edges, then fragment its faces, by that imprint.
    let mut split = Vec::with_capacity(healed.len());
    for (index, operand) in healed.iter().enumerate() {
        split.push(apply_edge_splits(operand, index as u8, &imprint)?);
    }
    let mut fragments = Vec::with_capacity(split.len());
    for (index, solid) in split.iter().enumerate() {
        fragments.push(fragment_solid(solid, index as u8, &imprint)?);
    }

    // 3. Select fragments in one pass against the set of other operands.
    let selected = select_fragments_nary(&fragments, &healed, operation, tolerance, debug)?;

    // 4. Assemble + normalize + validate, mirroring the binary tail.
    let solids: HashMap<u8, &BrepSolid> = split
        .iter()
        .enumerate()
        .map(|(index, solid)| (index as u8, solid))
        .collect();
    let solid = assemble_fragments(selected, &solids, &imprint, tolerance)?;
    let solid = if options.merge_coplanar_faces
        && std::env::var("BREP_COPLANAR_MERGE").as_deref() != Ok("0")
    {
        let merged = merge_same_surface_faces_excluding(
            &solid,
            tolerance,
            &options.keep_unmerged_name_substrs,
        )?;
        let merged = merge_curve_continuation_edges(&merged, tolerance).map_err(|refusal| KernelRefusal { stage: KernelStage::Validate, ..refusal })?;
        rejoin::rejoin_unless_referenced(merged, options, tolerance)
    } else {
        solid
    };
    let validation = solid.validate_detailed(&policy);
    if !validation.issues.is_empty() {
        return Err(KernelRefusal::new(
            RefusalClass::InvalidResultTopology { issues: validation.issues.len() as u32 },
            KernelStage::Validate,
            format!(
            "boolean_operation_nary: invalid result: {:?}",
            validation.issues
        )));
    }
    if debug {
        if let Ok(report) =
            crate::oracle::boolean_semantic_disagreement_nary(&healed, operation, &solid, 300)
        {
            if report.is_flagged() {
                eprintln!(
                    "[oracle] n-ary semantic disagreement rate {:.4} ({} of {} decidable points); sample {:?}",
                    report.disagreement_rate,
                    report.disagreements.len(),
                    report.considered,
                    report.sample_disagreement()
                );
            }
        }
    }
    Ok(solid)
}

/// TANGENT-NODE ATTRIBUTION.
///
/// The imprint's second-order filter decides which tangential contacts are
/// isolated NODES the arrangement can carve, and marches those instead of
/// refusing. It cannot decide a THIRD-order singularity — where the transverse
/// branch is tangent to a contact curve, every second-order quantity vanishes
/// together and the contact reads as a clean saddle at every stopping rule
/// (`imprint/tangent_contact.rs`). The assembly is the authority on whether an
/// admitted node was really imprintable, so a tear downstream of one is
/// reported AGAINST THE NODE rather than as an anonymous degeneracy: same
/// refusal class the whole tangent-node family carries, naming the point.
///
/// Only classes that are ALREADY perturbation-eligible are re-attributed, so a
/// refusal that could never take the ladder still cannot. The stage's own
/// message — the one-use edges, the genus — is NOT carried into the refusal:
/// `tangent_node_assembly` and `torus_torus_singular_refusal` hold the
/// tangent-node refusal to naming its cause without the arrangement's
/// symptom text ("open between", "genus", "pcurve"), the policy the class
/// exists for. It is printed under `BREP_DEBUG_BOOL` instead, so what the
/// assembly actually read is still there for a reader of the trace.
///
/// It does decide WHICH refusals are retried, though, and since 2026-09-13 that
/// matters: `boolean_operation`'s failure arm gates `TangentNodeSingularity` out
/// of the perturbation ladder (the TANGENT-NODE GATE there) and reads the class
/// AFTER this rewrite. So a tear downstream of an ADMITTED node — whatever class
/// the pipeline actually raised — declines the ladder along with the tangent-node
/// family proper. That is the conservative direction and the same claim the class
/// makes (a tear after a node is the node proving unimprintable). Anything added
/// to this rewrite widens that gate: read the gate when you touch the
/// attribution.
fn attribute_to_tangent_node(error: KernelRefusal, nodes: &[Vec3]) -> KernelRefusal {
    let Some(node) = nodes.first() else {
        return error;
    };
    if !error.class.perturbation_eligible()
        || matches!(error.class, RefusalClass::TangentNodeSingularity)
    {
        return error;
    }
    if std::env::var("BREP_DEBUG_BOOL").is_ok() {
        eprintln!(
            "boolean: tear downstream of the tangent node at ({:.6},{:.6},{:.6}) attributed to it; the stage read: {}",
            node.x, node.y, node.z, error.message
        );
    }
    KernelRefusal::new(
        RefusalClass::TangentNodeSingularity,
        error.stage,
        format!(
            "boolean: unsupported singular/tangent-node surface intersection \
             at the tangent node at ({:.6},{:.6},{:.6}): the section through it did not \
             assemble into a closed shell",
            node.x, node.y, node.z
        ),
    )
}

/// The tangent-curve lane's attribution: an ASSEMBLY refusal whose faces the
/// imprint found touching along a CURVE is reported against that contact, by
/// its faces and its ends, in the tangent-node class — the arrangement's own
/// message (an open loop, a one-use edge) is the symptom, the contact is the
/// cause, and the class keeps the refusal out of the perturbation ladder
/// exactly as an admitted node's tear is kept out
/// (`attribute_to_tangent_node`). The original message is carried along so
/// nothing the stage said is hidden.
///
/// REACH: the faces the refusal names, and only those. `origins` maps the
/// assembled result's face ids (minted one per selected fragment, from 1, in
/// order) back to operand faces; the ids are read off the message's
/// `face <id>` (validate's open-loop issues) and `on faces [<id> '…`
/// (one-use edges) entries, and the rewrite happens only when one of them is
/// a face the section INVOLVES (`TangentSectionRecord::faces`: the carriers'
/// faces and the faces across the edges its ends lie on). A refusal naming no
/// face, or only faces the contact does not touch — a model with a benign
/// recorded contact (a blind pocket touching a wall) that fails elsewhere —
/// keeps its class and enters the ladder as it always did. With
/// `ImprintOptions::tangent_curve_lane` off the sections are empty and
/// nothing is rewritten.
pub fn attribute_to_tangent_section(
    error: KernelRefusal,
    sections: &[crate::imprint::TangentSectionRecord],
    origins: &[FaceKey],
) -> KernelRefusal {
    if sections.is_empty()
        || !error.class.perturbation_eligible()
        || matches!(error.class, RefusalClass::TangentNodeSingularity)
    {
        return error;
    }
    let named = refusal_result_faces(&error.message)
        .into_iter()
        .filter_map(|id| origins.get(id.checked_sub(1)? as usize).copied())
        .collect::<Vec<FaceKey>>();
    // A record the lane's own sites minted before one the grazing-pierce
    // path minted (`TangentSectionRecord::grazing`): the sprocket slit star's
    // cutter names its traced contact, not the pierce of a neighbouring pair.
    let involving = |fallback: bool| {
        sections.iter().find(|section| {
            section.grazing.is_empty() != fallback && named.iter().any(|face| section.faces.contains(face))
        })
    };
    let Some(section) = involving(false).or_else(|| involving(true)) else {
        return error;
    };
    // A clipped end sits on a trim at round-off, and `-0.000000` names no
    // place: print what six decimals can show.
    let at = |point: Vec3| {
        let shown = |value: f64| if value.abs() < 5e-7 { 0.0 } else { value };
        format!("({:.6},{:.6},{:.6})", shown(point.x), shown(point.y), shown(point.z))
    };
    // The grazing pierces (`imprint/driver.rs`, 2026-10-17): trim edges that
    // pierce the other carrier inside both trims below the seed gate while
    // the pair marched nothing. A record the lane built stands on its contact
    // and carries them as a note; a record standing on the pierces ALONE
    // (its `start` IS the first pierce) is named by them.
    let pierces = (!section.grazing.is_empty()).then(|| {
        let shown = section.grazing.len().min(4);
        let mut listed = section.grazing[..shown]
            .iter()
            .map(|(point, tangency)| format!("{} at |tan·n| = {tangency:.4}", at(*point)))
            .collect::<Vec<_>>()
            .join(", ");
        if section.grazing.len() > shown {
            listed.push_str(&format!(" and {} more", section.grazing.len() - shown));
        }
        listed
    });
    let pierce_only = section.pierce_only();
    let shape = if let (true, Some(pierces)) = (pierce_only, pierces.as_deref()) {
        format!(
            "cross at a grazing angle: a trim edge pierces the other carrier inside both \
             trims at {pierces}, below the marcher's start gate, and no transverse section was \
             marched between the pierces — a near-tangent crossing born on a tangent contact \
             (a third-order singularity) the imprint does not build yet"
        )
    } else if section.planar {
        format!(
            "touch along a tangent curve from {} to {} (length {:.6}, {} stations{}) which the \
             imprint does not build yet",
            at(section.start),
            at(section.end),
            section.length,
            section.stations,
            if section.closed { ", closed" } else { "" }
        )
    } else {
        format!(
            "touch along a curve through {} with neither carrier planar, a curved/curved \
             tangent contact the lane does not trace yet",
            at(section.start)
        )
    };
    let shape = match (pierce_only, pierces) {
        (false, Some(pierces)) => format!(
            "{shape}; a trim edge also pierces the other carrier inside both trims at {pierces}, \
             below the marcher's start gate, with no transverse section marched"
        ),
        _ => shape,
    };
    let involved = named
        .iter()
        .filter(|face| section.faces.contains(face))
        .map(|face| format!("{}/{}", face.operand, face.face_id))
        .collect::<Vec<_>>()
        .join(", ");
    KernelRefusal::new(
        RefusalClass::TangentNodeSingularity,
        error.stage,
        format!(
            "boolean: unsupported tangent-curve surface intersection between faces {}/{} and {}/{}: \
             the carriers {shape}; the refusal names face(s) {involved} the contact involves; {}",
            section.first.operand, section.first.face_id, section.second.operand, section.second.face_id,
            error.message
        ),
    )
}

/// The RESULT face ids an assembly refusal names: every `face <id>` (the
/// validator's "loop … of face `<id>` is open …") and every `<id> '` entry of an
/// `on faces [...]` list (the one-use edge census). Nothing else in the
/// message is read.
pub fn refusal_result_faces(message: &str) -> Vec<u64> {
    let mut ids = Vec::new();
    let digits_after = |text: &str| -> Option<u64> {
        let end = text.find(|c: char| !c.is_ascii_digit()).unwrap_or(text.len());
        text[..end].parse().ok()
    };
    for (index, _) in message.match_indices("face ") {
        if let Some(id) = digits_after(&message[index + "face ".len()..]) {
            ids.push(id);
        }
    }
    if let Some(start) = message.find("on faces [") {
        let rest = &message[start + "on faces [".len()..];
        let list = &rest[..rest.find(']').unwrap_or(rest.len())];
        for entry in list.split(", ") {
            let entry = entry.trim_start();
            if let Some(id) = digits_after(entry) {
                if entry[id.to_string().len()..].starts_with(" '") {
                    ids.push(id);
                }
            }
        }
    }
    ids.sort_unstable();
    ids.dedup();
    ids
}

pub fn boolean_operation_with_diagnostics(
    first: &BrepSolid,
    second: &BrepSolid,
    operation: BooleanOperation,
    options: &BooleanOptions,
) -> Result<KernelOutcome<BrepSolid>, KernelRefusal> {
    if let Some(scaled) = WorkingScale::of(&[first, second], options)? {
        return scaled.run(
            || boolean_operation_with_diagnostics(first, second, operation, options),
            || {
                let mut outcome = boolean_operation_with_diagnostics(&scaled.solids[0], &scaled.solids[1], operation, &scaled.options)
                    .map_err(|error| scaled.refusal(error))?;
                outcome.value = scaled.back(&outcome.value)?;
                outcome.diagnostics.count_n("working_scale.fired", 1);
                outcome.diagnostics.measure_max("working_scale.factor", scaled.factor);
                Ok(outcome)
            },
        );
    }
    let _account = profile::Scope::new("boolean.other");
    // A whole ball turned onto the line the other operand cuts it about
    // (`ball_align`): its sections become latitudes.
    let (first_aligned, second_aligned) = ball_align::align_balls(first, second, operation, options.tolerance);
    let (first, second) = (first_aligned.as_ref(), second_aligned.as_ref());
    let closed_inputs = matches!(std::env::var("BREP_CLOSED_OPERAND_INPUT").as_deref(), Err(_) | Ok("1") | Ok("on"));
    if let (true, Some(later)) = (reseat::seam_reseat_enabled(), &options.seam_reseat_later_references) {
        let started = Instant::now();
        let (first, second) = reseat::reseat_tangent_seams(first, second, later, options.tolerance);
        profile_stage("seam_prepare_ms", started.elapsed().as_secs_f64() * 1000.0);
        return boolean_operation_with_diagnostics_impl(&first, &second, operation, options, closed_inputs).map(without_sliver_shells);
    }
    // CYLINDER CONTACTS (`reseat::seam_reseat_cylinders_mode`): with no later
    // references stated, only the cylinder-against-cylinder re-seat runs.
    let cylinders = reseat::seam_reseat_cylinders_mode();
    if reseat::seam_reseat_enabled() && cylinders != crate::env_read::Mode::Off {
        let kinds = reseat::ContactKinds { planes: false, cylinders: true };
        let volume = |result: &Result<KernelOutcome<BrepSolid>, KernelRefusal>| {
            result.as_ref().ok().and_then(|outcome| crate::solid_mass_properties(&outcome.value).ok()).map(|mass| mass.volume)
        };
        if reseat::seam_reseat_retry_enabled() {
            // RETRY: the operands as given first; the re-seated ones (with the
            // tangent-cylinder cosurface exchange) only when that refuses, so a
            // body built without a re-seat stays the operand of the next cut as
            // it would be without the switch. `check` reports what the retry
            // would give and returns the first attempt.
            let plain = boolean_operation_with_diagnostics_impl(first, second, operation, options, closed_inputs).map(without_sliver_shells);
            if plain.is_ok() {
                return plain;
            }
            let (reseated_first, reseated_second) = reseat::reseat_tangent_seams_of(first, second, &[], options.tolerance, kinds);
            let moved = matches!(reseated_first, std::borrow::Cow::Owned(_)) || matches!(reseated_second, std::borrow::Cow::Owned(_));
            if !moved {
                return plain;
            }
            let reseated = crate::imprint::driver::with_tangent_cylinder_filter(|| {
                boolean_operation_with_diagnostics_impl(&reseated_first, &reseated_second, operation, options, closed_inputs).map(without_sliver_shells)
            });
            if cylinders == crate::env_read::Mode::Check {
                eprintln!(
                    "boolean.seam_reseat_cylinders check: plain refused, reseated retry {} volume {:?}",
                    if reseated.is_ok() { "built" } else { "refused" },
                    volume(&reseated)
                );
                return plain;
            }
            return match reseated {
                Ok(mut outcome) => {
                    outcome.diagnostics.count_n("seam_reseat_cylinders.retry_built", 1);
                    Ok(outcome)
                }
                Err(_) => plain,
            };
        }
        let (reseated_first, reseated_second) = reseat::reseat_tangent_seams_of(first, second, &[], options.tolerance, kinds);
        let moved = matches!(reseated_first, std::borrow::Cow::Owned(_)) || matches!(reseated_second, std::borrow::Cow::Owned(_));
        if moved {
            let reseated = boolean_operation_with_diagnostics_impl(&reseated_first, &reseated_second, operation, options, closed_inputs).map(without_sliver_shells);
            if cylinders == crate::env_read::Mode::On {
                return reseated;
            }
            let plain = boolean_operation_with_diagnostics_impl(first, second, operation, options, closed_inputs).map(without_sliver_shells);
            eprintln!(
                "boolean.seam_reseat_cylinders check: plain {} volume {:?}, reseated {} volume {:?}",
                if plain.is_ok() { "built" } else { "refused" },
                volume(&plain),
                if reseated.is_ok() { "built" } else { "refused" },
                volume(&reseated)
            );
            return plain;
        }
    }
    boolean_operation_with_diagnostics_impl(first, second, operation, options, closed_inputs).map(without_sliver_shells)
}

/// [`drop_sliver_shells`] on a boolean's outcome, counted in its diagnostics.
fn without_sliver_shells(mut outcome: KernelOutcome<BrepSolid>) -> KernelOutcome<BrepSolid> {
    let dropped = drop_sliver_shells(&mut outcome.value);
    if dropped > 0 {
        outcome.diagnostics.count_n("sliver_shell.dropped", dropped as u64);
    }
    outcome
}

pub fn boolean_operation_with_diagnostics_impl(
    first: &BrepSolid,
    second: &BrepSolid,
    operation: BooleanOperation,
    options: &BooleanOptions,
    closed_inputs: bool,
) -> Result<KernelOutcome<BrepSolid>, KernelRefusal> {
    // One coedge-check memo per operation: the
    // assembly, face-merge and closing validations re-read coedges unchanged.
    let _coedge_memo = crate::topology::CoedgeMemoScope::open(true);
    // Plan 1.6: only a declared Sheet enters the trim path. The unchanged
    // Solid path continues below; no kind is inferred from failed closure.
    if matches!(operation, BooleanOperation::Intersect)
        && [first, second].iter().any(|body| body.kind == crate::BodyKind::Sheet)
        && match std::env::var("BREP_SHEET_TRIM").as_deref() {
            Ok("1") => true,
            Ok("0") => false,
            _ => options.sheet_trim,
        }
    {
        return sheet::trim(first, second, options);
    }
    // OPERAND-KIND GATE. A body that DECLARES itself a Sheet has no
    // inside, so a union or subtract with it has no solid answer; the
    // survey (record 2026-10-12, table 2) measured the arrangement
    // returning a validating solid of volume 3.5 when the sheet's opening
    // lay outside the other operand. Read from the declaration only,
    // before any arrangement work (nothing infers a kind from the edge
    // census; an undeclared open body is the closure gate's below).
    // Intersect is not gated here.
    if matches!(operation, BooleanOperation::Union | BooleanOperation::Subtract) {
        if let Some(operand) = [first, second]
            .iter()
            .position(|solid| solid.kind == crate::BodyKind::Sheet)
        {
            return Err(KernelRefusal::input(
                KernelStage::Collect,
                "sheet_operand",
                format!(
                    "boolean: operand {operand} is a Sheet, which encloses no solid; union and \
                     subtract of a sheet are refused; Intersect trims a Sheet by a Solid"
                ),
            ));
        }
    }
    // Inside/outside is undefined for an open operand, even if its opening
    // misses the other operand and the constructed result happens to validate.
    // Count existing edge uses, without modifying/healing either operand.
    // Keep failures on their original path and preserve their class/evidence;
    // the existing perturbation gate already diagnoses open failed operands.
    let mut outcome = boolean_operation_with_diagnostics_core(first, second, operation, options)?;
    let closure_started = closed_inputs.then(Instant::now);
    if closed_inputs {
        for (operand, solid) in [first, second].into_iter().enumerate() {
            let open = one_use_edges(solid);
            if open > 0 {
                return Err(KernelRefusal::input(KernelStage::Collect, "operand_closure", format!(
                    "boolean: operand {operand} is not a closed shell: {open} of its edges bound only one face, so its inside is undefined"
                )));
            }
        }
    }
    if let Some(started) = closure_started {
        let ms = started.elapsed().as_secs_f64() * 1000.0;
        profile_stage("operand_closure_ms", ms);
        outcome.diagnostics.measure_max("timing.operand_closure_ms", ms);
    }
    Ok(outcome)
}

fn boolean_operation_with_diagnostics_core(
    first: &BrepSolid, second: &BrepSolid, operation: BooleanOperation, options: &BooleanOptions,
) -> Result<KernelOutcome<BrepSolid>, KernelRefusal> {

    // The translation control (`BREP_TRANSLATION_CONTROL`, `brep/recentre.rs`):
    // the outermost entry runs the operation on translated operands.
    if let Some(offset) = crate::recentre::control_offset_at_entry() {
        return boolean_operation_with_diagnostics_at_offset(first, second, operation, options, offset);
    }
    // The recentre guard: operands far from the origin in their own
    // units run at the origin.
    let mut bounds = operand_bounds(first);
    bounds.include(operand_bounds(second));
    if let Some(offset) = crate::recentre::guard_offset(bounds) {
        crate::recentre::profile_fired("boolean", offset, bounds);
        let mut outcome = boolean_translated(first, second, operation, options, offset, crate::recentre::EntryKind::Guard)?;
        outcome.diagnostics.count_n("recentre.fired", 1);
        outcome.diagnostics.measure_max("recentre.distance", offset.length());
        return Ok(outcome);
    }
    let mut tangent_nodes = Vec::new();
    let (mut outcome, soundness_candidates) =
        boolean_pipeline(first, second, operation, options, &mut tangent_nodes)
            .map_err(|error| attribute_to_tangent_node(error, &tangent_nodes))?;
    // The assembly's single exit. A boolean can hand back a body whose walls
    // pass through each other while `validate()` is silent and the volume
    // measures right, which is exactly the failure a regularized boolean must
    // not report as success. A crossing the OPERANDS already had is theirs and
    // passes through — the defect came in with the input and refusing it here
    // would blame the wrong operation — so the operands are only scanned when
    // the result is flagged, and the common case pays for one scan. The
    // acceptance's refusal is typed at its origin (an `UnsoundResult` naming
    // the defect and the faces); it was classed `Internal` here while
    // `accept_sound_against` still returned text. Timed as its own stage: the
    // record's phase split read only the pipeline's `timing.*` measures, and
    // on a 108-face disjoint union this scan was 185 of 226 ms end to end.
    let stage_started = Instant::now();
    let accepted = match &soundness_candidates {
        // Pass-through: the faces the operation touched, their
        // neighbours and every face whose box reaches the other operand.
        Some(candidates) => crate::accept_sound_against_scoped(&[first, second], outcome.value, "boolean", candidates),
        None => crate::accept_sound_against(&[first, second], outcome.value, "boolean"),
    };
    let stage_ms = stage_started.elapsed().as_secs_f64() * 1_000.0;
    profile_stage("soundness_ms", stage_ms);
    outcome.diagnostics.measure_max("timing.soundness_ms", stage_ms);
    outcome.value = accepted?;
    Ok(outcome)
}

/// [`boolean_operation_with_diagnostics`] with both operands translated by
/// `offset` for the operation and the result translated back. Refusal
/// messages name the translated coordinates.
pub fn boolean_operation_with_diagnostics_at_offset(
    first: &BrepSolid,
    second: &BrepSolid,
    operation: BooleanOperation,
    options: &BooleanOptions,
    offset: Vec3,
) -> Result<KernelOutcome<BrepSolid>, KernelRefusal> {
    boolean_translated(first, second, operation, options, offset, crate::recentre::EntryKind::Control)
}

/// The translated run behind the control and the guard.
fn boolean_translated(
    first: &BrepSolid,
    second: &BrepSolid,
    operation: BooleanOperation,
    options: &BooleanOptions,
    offset: Vec3,
    kind: crate::recentre::EntryKind,
) -> Result<KernelOutcome<BrepSolid>, KernelRefusal> {
    let _entry = crate::recentre::TranslatedEntry::open(kind);
    let translate = |solid: &BrepSolid, offset: Vec3| {
        crate::recentre::translate_solid(solid, offset).map_err(|message| {
            KernelRefusal::internal(
                KernelStage::Collect,
                "translation_control",
                format!("boolean: the translation control could not move an operand: {message}"),
            )
        })
    };
    let moved_first = translate(first, offset)?;
    let moved_second = translate(second, offset)?;
    let outcome = boolean_operation_with_diagnostics(&moved_first, &moved_second, operation, options)?;
    let value = translate(&outcome.value, offset.scale(-1.0))?;
    Ok(KernelOutcome { value, diagnostics: outcome.diagnostics })
}

/// Stage wall-clock line on stderr when `BREP_PROFILE` is set — the same switch
/// `imprint.profile` and `assemble.*_ms` use — so a REFUSED boolean, whose
/// `KernelDiagnostics` never reach a caller, still shows where its time went.
fn profile_stage(code: &str, ms: f64) {
    let row = match code {
        "heal_ms" => Some("boolean.operand_prepare"),
        "imprint_ms" => Some("boolean.imprint"),
        "edge_split_ms" => Some("boolean.edge_split"),
        "fragment_ms" => Some("boolean.fragment"),
        "select_ms" => Some("boolean.select"),
        "assemble_ms" => Some("boolean.assemble"),
        "face_merge_ms" => Some("boolean.face_merge"),
        "soundness_ms" => Some("boolean.soundness"),
        "operand_closure_ms" => Some("boolean.operand_closure"),
        "seam_prepare_ms" => Some("boolean.seam_prepare"),
        "final_validate_ms" => Some("boolean.final_validate"),
        _ => None,
    };
    if let Some(row) = row { profile::record(row, ms); }
    if crate::profile::profile_enabled() {
        profile_line!("boolean.{code}={ms:.2}");
    }
}

/// `BREP_BOOLEAN_DUMP` names a directory. Every binary boolean then writes its
/// stages there as JSON — the two split operands, the selected fragments, the
/// assembled solid and the merged one — numbered per process, so a probe can
/// read a face's loops at each stage without replaying the document.
struct BooleanStageDump {
    directory: std::path::PathBuf,
    serial: usize,
}

impl BooleanStageDump {
    fn open() -> Option<Self> {
        static SERIAL: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
        let directory = std::env::var("BREP_BOOLEAN_DUMP").ok().filter(|value| !value.is_empty())?;
        let serial = SERIAL.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        Some(Self { directory: directory.into(), serial })
    }

    fn write<T: Serialize + ?Sized>(&self, stage: &str, value: &T) {
        let path = self.directory.join(format!("{:03}-{stage}.json", self.serial));
        match serde_json::to_string(value) {
            Ok(text) => {
                if let Err(error) = std::fs::write(&path, text) {
                    eprintln!("BREP_BOOLEAN_DUMP {}: {error}", path.display());
                }
            }
            Err(error) => eprintln!("BREP_BOOLEAN_DUMP {stage}: {error}"),
        }
    }
}

/// The tolerance policy a binary boolean of `first` and `second` runs under.
pub fn boolean_policy(
    first: &BrepSolid,
    second: &BrepSolid,
    options: &BooleanOptions,
) -> Result<KernelTolerances, KernelRefusal> {
    let policy = options
        .tolerances
        .unwrap_or_else(|| KernelTolerances::for_pair(first, second, options.tolerance));
    policy.check().or_input(KernelStage::Collect, "tolerances")?;
    Ok(policy)
}

/// The operands as the imprint reads them: healed, with their band seams
/// normalized.
fn prepared_operands(
    first: &BrepSolid,
    second: &BrepSolid,
    policy: &KernelTolerances,
) -> Result<(BrepSolid, BrepSolid), KernelRefusal> {
    // Fuse-first operand healing (Lever A): before any intersection touches the
    // operands, snap each one's near-coincident / off-plane vertices to exact
    // and re-anchor the incident edge curves onto them, so noisy
    // near-degenerate input (points that should coincide but drifted, vertices
    // a few microns off a planar face) cannot tip a fixed downstream band over
    // the edge.  Healing is per-operand, validate-gated, and — because a clean
    // operand has no vertices within `heal_tol` and none off its planes — a
    // no-op that leaves the boolean output byte-identical on clean inputs.
    let stage_started = Instant::now();
    let mut first_owned = first.clone();
    let mut second_owned = second.clone();
    crate::heal::heal_operands(&mut first_owned, policy)?;
    crate::heal::heal_operands(&mut second_owned, policy)?;
    profile_stage("heal.operands_ms", stage_started.elapsed().as_secs_f64() * 1_000.0);
    // Then refit, on each operand's own carriers, any inherited trim off the
    // pcurve construction contract whose edge lies on every incident carrier
    // (`heal::refit_inherited_trims`; a byte-identical no-op without one).
    let refit_started = Instant::now();
    crate::heal::refit_inherited_trims_reused(&mut first_owned, policy)?;
    crate::heal::refit_inherited_trims_reused(&mut second_owned, policy)?;
    profile_stage("heal.refit_ms", refit_started.elapsed().as_secs_f64() * 1_000.0);
    if crate::profile::profile_enabled() {
        let (hits, misses) = crate::reading_memo::counts();
        profile_line!("boolean.reading_memo hits={hits} misses={misses}");
        let (served, stored) = crate::heal::refit_memo_counts_served_stored();
        profile_line!("boolean.refit_memo served={served} stored={stored}");
        let (served, stored, mismatches) = crate::heal::claim_record_memo_counts();
        profile_line!("boolean.claim_record_memo served={served} stored={stored} check_mismatches={mismatches}");
    }
    profile_stage("heal_ms", stage_started.elapsed().as_secs_f64() * 1_000.0);
    // Seamless full-period band faces (STEP import) break the seam-aware
    // imprint/arrangement machinery; normalize them to the seam-carrying
    // topology native periodic faces use. No-op on operands without such
    // faces. See `insert_periodic_band_seam_edges`.
    normalize_operand_band_seams(&mut first_owned)?;
    normalize_operand_band_seams(&mut second_owned)?;
    Ok((first_owned, second_owned))
}

fn boolean_imprint_options(options: &BooleanOptions, tolerance: f64) -> ImprintOptions {
    let mut imprint_options = options.imprint.clone();
    imprint_options.tolerance = tolerance;
    imprint_options
}

/// The two operands a binary boolean's arrangement reads: prepared as the
/// boolean prepares them, with every boundary edge split where the imprint's
/// sections meet it. These are the `NNN-first` and `NNN-second` stages
/// `BREP_BOOLEAN_DUMP` writes, not the operands at the boolean's entry.
///
/// A boolean must read the same on these as on the operands it split them
/// from: the splits are vertices its own imprint puts on those edges anyway.
/// Where it does not, an imprint lane is deciding on what the input happens to
/// have split, and that is a defect.
pub fn boolean_split_operands(
    first: &BrepSolid,
    second: &BrepSolid,
    options: &BooleanOptions,
) -> Result<(BrepSolid, BrepSolid), KernelRefusal> {
    if let Some(scaled) = WorkingScale::of(&[first, second], options)? {
        return scaled.run(
            || boolean_split_operands(first, second, options),
            || {
                let (split_first, split_second) = boolean_split_operands(&scaled.solids[0], &scaled.solids[1], &scaled.options)
                    .map_err(|error| scaled.refusal(error))?;
                Ok((scaled.back(&split_first)?, scaled.back(&split_second)?))
            },
        );
    }
    let policy = boolean_policy(first, second, options)?;
    let (first, second) = prepared_operands(first, second, &policy)?;
    let imprint = build_imprints(&first, &second, &boolean_imprint_options(options, policy.model))?;
    let (split_first, _) = apply_edge_splits_with_map(&first, 0, &imprint)?;
    let (split_second, _) = apply_edge_splits_with_map(&second, 1, &imprint)?;
    Ok((split_first, split_second))
}

/// Conservative bounds of an operand: its vertices and every face's control
/// net (a NURBS lies in the hull of its controls). A control net that cannot
/// be read bounds everything, so nothing is left out for want of a reading.
fn operand_bounds(solid: &BrepSolid) -> crate::Aabb {
    let mut bounds = crate::Aabb::from_points(solid.vertices.iter().map(|vertex| vertex.point));
    for face in solid.shells.iter().flat_map(|shell| &shell.faces) {
        match crate::Aabb::from_surface_controls(&face.surface) {
            Ok(face_bounds) => bounds.include(face_bounds),
            Err(_) => {
                return crate::Aabb {
                    minimum: Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY),
                    maximum: Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY),
                }
            }
        }
    }
    bounds
}

/// Row 12's gate: the boxes are separated or touch exactly in at least two
/// axes. A trimmed face lies in its carrier's control-net box when every
/// weight is positive, so a point interior to both operands would lie
/// strictly inside both boxes in every axis; two such axes leave a line or a
/// point of contact, which has no area to merge.
fn line_or_point_contact(first: &BrepSolid, second: &BrepSolid, bounds: [crate::Aabb; 2]) -> bool {
    apart_axes(bounds) >= 2 && plain_hull_solid(first) && plain_hull_solid(second)
}

/// One apart axis: no point can be interior to both operands (see
/// [`line_or_point_contact`]), though the boxes may share a face's area.
fn boxes_share_no_interior(first: &BrepSolid, second: &BrepSolid, bounds: [crate::Aabb; 2]) -> bool {
    apart_axes(bounds) >= 1 && plain_hull_solid(first) && plain_hull_solid(second)
}

/// The axes in which the boxes are separated or touch exactly (no band).
fn apart_axes([a, b]: [crate::Aabb; 2]) -> usize {
    let apart = |a_min: f64, a_max: f64, b_min: f64, b_max: f64| a_max <= b_min || b_max <= a_min;
    [
        apart(a.minimum.x, a.maximum.x, b.minimum.x, b.maximum.x),
        apart(a.minimum.y, a.maximum.y, b.minimum.y, b.maximum.y),
        apart(a.minimum.z, a.maximum.z, b.minimum.z, b.maximum.z),
    ]
    .into_iter()
    .filter(|&apart| apart)
    .count()
}

/// A plain solid whose faces lie in their control-net boxes: every weight positive.
fn plain_hull_solid(solid: &BrepSolid) -> bool {
    solid.kind == crate::BodyKind::Solid
        && solid.cells.is_empty()
        && solid.shells.iter().flat_map(|shell| &shell.faces).all(|face| {
            face.surface.control_points.iter().flatten().all(|control| control.w > 0.0 && control.w.is_finite())
        })
}

/// `BREP_SCOPE_PROVED_APART` (default on since the 2026-11-19 promotion;
/// `=0` scans every face whose box reaches the other operand): see
/// [`soundness_candidates`].
fn scope_proved_apart_on() -> bool {
    switch!(live "BREP_SCOPE_PROVED_APART", true)
}

/// The result faces the soundness scan reads under pass-through: the
/// examined faces (touched and their neighbours) and every other face whose
/// control-net box reaches the OTHER operand's bounds within `band`. A face
/// outside both is an operand's own, clear of everything the operation added.
///
/// PROVED APART (`BREP_SCOPE_PROVED_APART`): with `unproved` (the imprint's
/// [`crate::ImprintResultRecord::faces_in_unproved_pairs`]) and both operands
/// plain hull solids, a face whose box reaches the other operand is still
/// left out when the imprint PROVED it clear of every face of the other
/// operand (its box met none, or each pair it met was classified `Disjoint`
/// or proved apart by span hulls). A pair the imprint skipped, marched to
/// nothing, or could not prove keeps the face in. A whole-body scan of a body
/// nested inside the other operand's box (a closed hollow: the source less
/// its inward offset, untouched) read every face of both for that alone.
fn soundness_candidates(
    solid: &BrepSolid,
    scope: &AssemblyScope,
    bounds: [crate::Aabb; 2],
    band: f64,
    unproved: Option<&HashSet<crate::imprint::FaceKey>>,
) -> HashSet<u64> {
    let mut candidates = HashSet::default();
    for face in solid.shells.iter().flat_map(|shell| &shell.faces) {
        if scope.examined.contains(&face.id) {
            candidates.insert(face.id);
            continue;
        }
        let Some(&operand) = scope.operand_of.get(&face.id) else {
            candidates.insert(face.id);
            continue;
        };
        let other = bounds[usize::from(operand == 0)];
        match crate::Aabb::from_surface_controls(&face.surface) {
            Ok(face_bounds) if !face_bounds.intersects(other, band) => {}
            _ => {
                let proved_clear = unproved.is_some_and(|unproved| {
                    scope.source_of.get(&face.id).is_some_and(|key| !unproved.contains(key))
                });
                if !proved_clear {
                    candidates.insert(face.id);
                }
            }
        }
    }
    candidates
}


/// `BREP_TOUCHING_EMPTY` (default on; =0 refuses as before): an intersection
/// or difference of operands that only touch, which selects no fragment,
/// returns the empty solid on the sample witnesses and an inclusion-exclusion
/// volume of zero (`boolean_pipeline`). brep-conformance's touching families
/// (boxes face to face, flush pockets, contact along a line) refused with
/// "operation produced no boundary faces" where the regularized result is
/// empty.
fn touching_empty_enabled() -> bool {
    switch!("BREP_TOUCHING_EMPTY", true)
}

thread_local! {
    /// Set while a touching-empty proof reads another boolean of its pair, so
    /// that reading cannot itself be rescued as empty.
    static TOUCHING_PROOF: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
}

/// Whether a cylinder face of `first` and one of `second`, or a ball of each,
/// reach into each other by more than `tolerance`: the closed-form depth of
/// their lens (radii against the closest distance of the axes, at any angle,
/// or of the centres; outer or nested), counted only where the two faces'
/// boxes overlap by more than `tolerance` along every axis (two cylinders
/// meeting end to end through their caps do not count).
fn hair_deep_overlap(first: &BrepSolid, second: &BrepSolid, tolerance: f64) -> bool {
    use crate::AnalyticSurface as A;
    fn faces(solid: &BrepSolid) -> Vec<(&crate::AnalyticSurface, crate::spatial::Aabb, &crate::NurbsSurface, &crate::FaceRecord)> {
        solid
            .shells
            .iter()
            .flat_map(|shell| &shell.faces)
            .filter_map(|face| Some((face.surface.analytic()?, crate::Aabb::from_surface_controls(&face.surface).ok()?, &face.surface, face)))
            .collect()
    }
    // A face's extent along `axis`: its control points' (the hull holds it).
    let along = |surface: &crate::NurbsSurface, axis: Vec3| {
        surface.control_points.iter().flatten().filter_map(|control| control.point().ok()).fold((f64::INFINITY, f64::NEG_INFINITY), |(low, high), point| {
            let at = point.dot(axis);
            (low.min(at), high.max(at))
        })
    };
    let crossing = |distance: f64, ra: f64, rb: f64| {
        let outer = ra + rb - distance;
        let nested = distance - (ra - rb).abs();
        outer > tolerance && nested > tolerance
    };
    let boxes_overlap = |a: &crate::spatial::Aabb, b: &crate::spatial::Aabb| {
        (a.maximum.x.min(b.maximum.x) - a.minimum.x.max(b.minimum.x)) > tolerance
            && (a.maximum.y.min(b.maximum.y) - a.minimum.y.max(b.minimum.y)) > tolerance
            && (a.maximum.z.min(b.maximum.z) - a.minimum.z.max(b.minimum.z)) > tolerance
    };
    for (a, box_a, surface_a, face_a) in faces(first) {
        for (b, box_b, surface_b, face_b) in faces(second) {
            if !boxes_overlap(&box_a, &box_b) {
                continue;
            }
            match (a, b) {
                (A::RuledRevolution { frame: fa, rho0: a0, rho1: a1, .. }, A::RuledRevolution { frame: fb, rho0: b0, rho1: b1, .. })
                    if (a0 - a1).abs() <= tolerance && (b0 - b1).abs() <= tolerance =>
                {
                    let offset = fb.origin.sub(fa.origin);
                    let normal = fa.axis.cross(fb.axis);
                    // Parallel walls meet only where their extents along the
                    // axis overlap (two cylinders end to end touch at a cap).
                    if normal.length() <= 1e-12 {
                        let (low_a, high_a) = along(surface_a, fa.axis);
                        let (low_b, high_b) = along(surface_b, fa.axis);
                        if high_a.min(high_b) - low_a.max(low_b) <= tolerance {
                            continue;
                        }
                    }
                    let apart = if normal.length() <= 1e-12 {
                        offset.sub(fa.axis.scale(offset.dot(fa.axis))).length()
                    } else {
                        offset.dot(normal).abs() / normal.length()
                    };
                    if crossing(apart, a0.abs(), b0.abs()) {
                        return true;
                    }
                }
                (A::Sphere { frame: fa, radius: ra }, A::Sphere { frame: fb, radius: rb }) => {
                    // The two balls' lens meets the faces only along the
                    // circle where the spheres cross; a face that circle is
                    // proven off (a part of a cavity the circle passes
                    // beside) holds no lens, however deep the carriers' is.
                    if crossing(fb.origin.sub(fa.origin).length(), *ra, *rb)
                        && !circle_off_face(face_a, first, (fa.origin, *ra), (fb.origin, *rb), tolerance)
                        && !circle_off_face(face_b, second, (fb.origin, *rb), (fa.origin, *ra), tolerance)
                    {
                        return true;
                    }
                }
                _ => {}
            }
        }
    }
    false
}

/// Whether the circle where the sphere `own` (centre, radius), carrying
/// `face`, crosses the sphere `other` is proven to lie off `face`: every edge
/// of the face stands clear of `other` by more than `tolerance` (its control
/// hull, halved where it does not decide, wholly inside or wholly outside),
/// so the circle, which lies on `other`, never meets the face's boundary and
/// is wholly on the face or wholly off it; and one point of it is outside the
/// face's trim. A face with no trim, or anything unproven, is not off.
fn circle_off_face(face: &crate::FaceRecord, solid: &BrepSolid, own: (Vec3, f64), other: (Vec3, f64), tolerance: f64) -> bool {
    use crate::classification::{parameter_point_in_face, PolygonClass};
    let ((ca, ra), (cb, rb)) = (own, other);
    if face.loops.iter().all(|lp| lp.coedges.is_empty()) {
        return false;
    }
    // A curve's points on [t0, t1] all inside or all outside the other ball:
    // its control points (positive weights: the hull holds the curve) all
    // within `rb - tolerance` of the centre, or their box farther than
    // `rb + tolerance` from it; otherwise each half, a few times over.
    fn clear(curve: &crate::NurbsCurve, depth: u32, centre: Vec3, radius: f64, tolerance: f64) -> bool {
        let points: Option<Vec<Vec3>> = curve.control_points.iter().map(|control| if control.w > 0.0 { control.point().ok() } else { None }).collect();
        let Some(points) = points else { return false };
        if points.iter().all(|point| point.sub(centre).length() < radius - tolerance) {
            return true;
        }
        let (mut low, mut high) = (Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY), Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY));
        for point in &points {
            low = Vec3::new(low.x.min(point.x), low.y.min(point.y), low.z.min(point.z));
            high = Vec3::new(high.x.max(point.x), high.y.max(point.y), high.z.max(point.z));
        }
        let near = Vec3::new(centre.x.clamp(low.x, high.x), centre.y.clamp(low.y, high.y), centre.z.clamp(low.z, high.z));
        if near.sub(centre).length() > radius + tolerance {
            return true;
        }
        if depth == 0 {
            return false;
        }
        let Ok([t0, t1]) = curve.domain() else { return false };
        match curve.split(0.5 * (t0 + t1)) {
            Ok((left, right)) => clear(&left, depth - 1, centre, radius, tolerance) && clear(&right, depth - 1, centre, radius, tolerance),
            Err(_) => false,
        }
    }
    for coedge in face.loops.iter().flat_map(|lp| &lp.coedges) {
        let Some(edge) = solid.edges.iter().find(|edge| edge.id == coedge.edge_id) else { return false };
        let Ok([d0, d1]) = edge.curve.domain() else { return false };
        // The edge's own span of its curve.
        let mut span = edge.curve.clone();
        if edge.t1 < d1 {
            match span.split(edge.t1) {
                Ok((left, _)) => span = left,
                Err(_) => return false,
            }
        }
        if edge.t0 > d0 {
            match span.split(edge.t0) {
                Ok((_, right)) => span = right,
                Err(_) => return false,
            }
        }
        if !clear(&span, 10, cb, rb, tolerance) {
            return false;
        }
    }
    let axis = cb.sub(ca);
    let distance = axis.length();
    let Ok(n) = axis.normalized() else { return false };
    let along = (distance * distance + ra * ra - rb * rb) / (2.0 * distance);
    let across = ra * ra - along * along;
    let Ok(side) = n.perpendicular() else { return false };
    if !(across > 0.0) {
        return false;
    }
    let point = ca.add(n.scale(along)).add(side.scale(across.sqrt()));
    let Ok(projection) = crate::project_point_to_surface(&face.surface, point) else { return false };
    matches!(parameter_point_in_face(face, crate::Vec2 { x: projection.u, y: projection.v }, 1e-9), Ok(PolygonClass::Outside))
}


/// The empty solid a legitimately empty boolean returns.
fn empty_solid() -> BrepSolid {
    BrepSolid {
        mass_properties_cache: Default::default(),
        cells: Default::default(),
        id: 0,
        vertices: Vec::new(),
        edges: Vec::new(),
        shells: Vec::new(),
        genus: 0,
        kind: crate::BodyKind::Solid,
    }
}

/// The two operands as one solid, shell by shell: ids minted afresh in the
/// assembler's order (the first operand's faces, then the second's), the
/// second operand's face names suffixed where they clash with the first's
/// as the assembler suffixes a split face, geometry copied byte for byte,
/// genus the sum. Nothing is welded: the operands are disjoint.
fn concatenate_disjoint(first: &BrepSolid, second: &BrepSolid) -> BrepSolid {
    let mut vertices = Vec::new();
    let mut edges = Vec::new();
    let mut shells = Vec::new();
    let mut used_face_names = HashSet::<String>::default();
    let (mut next_face_id, mut next_loop_id, mut next_coedge_id) = (1u64, 1u64, 1u64);
    for operand in [first, second] {
        let vertex_base = vertices.len() as u64;
        let edge_base = edges.len() as u64;
        let mut vertex_ids: HashMap<u64, u64> = HashMap::default();
        for (index, vertex) in operand.vertices.iter().enumerate() {
            let id = vertex_base + index as u64 + 1;
            vertex_ids.insert(vertex.id, id);
            vertices.push(VertexRecord { id, point: vertex.point });
        }
        let mut edge_ids: HashMap<u64, u64> = HashMap::default();
        for (index, edge) in operand.edges.iter().enumerate() {
            let id = edge_base + index as u64 + 1;
            edge_ids.insert(edge.id, id);
            edges.push(EdgeRecord {
                id,
                curve: edge.curve.clone(),
                t0: edge.t0,
                t1: edge.t1,
                start_vertex_id: vertex_ids.get(&edge.start_vertex_id).copied().unwrap_or(edge.start_vertex_id),
                end_vertex_id: vertex_ids.get(&edge.end_vertex_id).copied().unwrap_or(edge.end_vertex_id),
                degenerate: edge.degenerate,
                name: edge.name.clone(),
            });
        }
        for shell in &operand.shells {
            let mut faces = Vec::with_capacity(shell.faces.len());
            for face in &shell.faces {
                let loops = face
                    .loops
                    .iter()
                    .map(|loop_record| {
                        let coedges = loop_record
                            .coedges
                            .iter()
                            .map(|coedge| {
                                let id = next_coedge_id;
                                next_coedge_id += 1;
                                CoedgeRecord {
                                    id,
                                    edge_id: edge_ids.get(&coedge.edge_id).copied().unwrap_or(coedge.edge_id),
                                    forward: coedge.forward,
                                    pcurve: coedge.pcurve.clone(),
                                }
                            })
                            .collect();
                        let id = next_loop_id;
                        next_loop_id += 1;
                        LoopRecord { id, coedges }
                    })
                    .collect();
                let name = face.name.clone().map(|base| {
                    if used_face_names.insert(base.clone()) {
                        base
                    } else {
                        let mut suffix = 1usize;
                        loop {
                            let candidate = format!("{base}_{suffix}");
                            if used_face_names.insert(candidate.clone()) {
                                break candidate;
                            }
                            suffix += 1;
                        }
                    }
                });
                faces.push(FaceRecord {
                    id: next_face_id,
                    surface: face.surface.clone(),
                    same_sense: face.same_sense,
                    loops,
                    name,
                });
                next_face_id += 1;
            }
            shells.push(ShellRecord { id: shells.len() as u64 + 1, faces });
        }
    }
    BrepSolid {
        mass_properties_cache: Default::default(),
        cells: Default::default(),
        id: 1,
        vertices,
        edges,
        shells,
        genus: first.genus + second.genus,
        kind: crate::BodyKind::Solid,
    }
}

/// The binary boolean. The second value is the soundness scan's candidate
/// faces under pass-through (`None` reads the whole result).
fn boolean_pipeline(
    first: &BrepSolid,
    second: &BrepSolid,
    operation: BooleanOperation,
    options: &BooleanOptions,
    tangent_nodes: &mut Vec<Vec3>,
) -> Result<(KernelOutcome<BrepSolid>, Option<HashSet<u64>>), KernelRefusal> {
    let _work = crate::imprint::work_profile::Scope::open("boolean");
    let replaced_operands = operand_state::replace_operands(first, second, operation, options);
    let (first, second) = match &replaced_operands {
        Some((first, second)) => (first, second),
        None => (first, second),
    };
    let policy = boolean_policy(first, second, options)?;
    let tolerance = policy.model;
    // DISJOINT FAST PATH. A box test on the control-net hulls, not "no
    // section evidence": a tool inside the other operand also leaves no
    // evidence and must stay on the pipeline, where a union is the outer
    // operand, a subtract the outer with a void shell and an intersect the
    // tool. Operands clear of each other by more than the search band can
    // touch nothing, so nothing is prepared, imprinted, assembled or
    // scanned: a union is the two operands side by side, a subtract the
    // first operand, an intersect the legitimate-empty result below. The
    // soundness scan is skipped as well (an empty candidate set): the
    // operands cannot cross each other, and a crossing an operand carried in
    // passes through the full acceptance unrepaired exactly the same.
    //
    // CONTACT FAST PATH (row 12): boxes that only meet along a line or at a
    // point, read exactly; see `line_or_point_contact`.
    let bounds = [operand_bounds(first), operand_bounds(second)];
    let disjoint = disjoint_fast_path(options) && !bounds[0].intersects(bounds[1], policy.sew_search);
    let contact = !disjoint
        && contact_fast_path(options)
        && options.seam_reseat_later_references.is_some()
        && line_or_point_contact(first, second, bounds);
    // One apart axis already leaves no common interior, so an intersect is
    // empty even where the boxes share a face's area (which is why a union or
    // a subtract stays on the pipeline). Nothing survives to carry an id, so
    // no caller has to say what follows.
    let contact = contact
        || (!disjoint
            && matches!(operation, BooleanOperation::Intersect)
            && contact_fast_path(options)
            && boxes_share_no_interior(first, second, bounds));
    if disjoint || contact {
        let operation_started = Instant::now();
        let mut diagnostics = KernelDiagnostics::default();
        diagnostics.count_n(
            "collect.faces",
            first.shells.iter().chain(&second.shells).map(|shell| shell.faces.len() as u64).sum(),
        );
        diagnostics.count(if contact { "boolean.contact_fast_path" } else { "boolean.disjoint_fast_path" });
        diagnostics.measure_max("tolerance.model", policy.model);
        diagnostics.measure_max("tolerance.sew_search", policy.sew_search);
        let value = match operation {
            BooleanOperation::Union => concatenate_disjoint(first, second),
            BooleanOperation::Subtract => first.clone(),
            BooleanOperation::Intersect => {
                diagnostics.event(
                    DiagnosticSeverity::Info,
                    KernelStage::Select,
                    "boolean.empty_result",
                    format!("{operation:?} of witnessed-non-overlapping operands is empty"),
                );
                empty_solid()
            }
        };
        let stage_ms = operation_started.elapsed().as_secs_f64() * 1_000.0;
        profile_stage("disjoint_fast_path_ms", stage_ms);
        diagnostics.measure_max("timing.total_ms", stage_ms);
        return Ok((KernelOutcome { value, diagnostics }, Some(HashSet::default())));
    }
    let heal_started = Instant::now();
    let (first_owned, second_owned) = prepared_operands(first, second, &policy)?;
    let heal_ms = heal_started.elapsed().as_secs_f64() * 1_000.0;
    let first = &first_owned;
    let second = &second_owned;

    let mut diagnostics = KernelDiagnostics::default();
    // Operand preparation ran before `timing.total_ms` opens and was absent
    // from every phase split; it is 1.6 of a disjoint box union's 7.2 ms.
    diagnostics.measure_max("timing.heal_ms", heal_ms);
    // Every pcurve fit made while this scope is open — the imprint's sections,
    // the assembler's rebuilt trims, the merge's re-fits — is tallied, so the
    // outcome can say whether every curved trim reached its floor instead of
    // letting a sample-ceiling exit pass for a met one (`pcurve.unmet_floor`).
    let fit_scope = crate::PcurveFitScope::open();
    // And every POLYLINE fit: a marched section's interpolant is measured
    // against the tolerance it was given, so `fit.polyline.unmet_tolerance`
    // reports a section that could not reach it (`geometry/fit.rs`).
    let polyline_scope = crate::PolylineFitScope::open();
    let operation_started = Instant::now();
    diagnostics.count_n(
        "collect.faces",
        first
            .shells
            .iter()
            .chain(&second.shells)
            .map(|shell| shell.faces.len() as u64)
            .sum(),
    );
    diagnostics.measure_max("tolerance.model", policy.model);
    diagnostics.measure_max("tolerance.sew_search", policy.sew_search);

    let imprint_options = boolean_imprint_options(options, tolerance);
    let stage_started = Instant::now();
    let imprint = build_imprints(first, second, &imprint_options)?;
    tangent_nodes.extend_from_slice(&imprint.tangent_nodes);
    if std::env::var("BREP_DEBUG_BOOL").is_ok() {
        for piece in &imprint.pieces {
            let start = piece.curve.evaluate(piece.t0);
            let end = piece.curve.evaluate(piece.t1);
            eprintln!(
                "piece {} supports={:?} t=[{:.6},{:.6}] start={:?} end={:?}",
                piece.id, piece.support_faces, piece.t0, piece.t1, start, end
            );
        }
        for entry in &imprint.by_face {
            eprintln!(
                "by_face operand={} face={} pieces={:?}",
                entry.operand, entry.face_id, entry.piece_ids
            );
        }
        for split in &imprint.edge_splits {
            eprintln!(
                "edge_split operand={} edge={} params={:?}",
                split.operand, split.edge_id, split.parameters
            );
        }
    }
    let stage_ms = stage_started.elapsed().as_secs_f64() * 1_000.0;
    profile_stage("imprint_ms", stage_ms);
    diagnostics.measure_max("timing.imprint_ms", stage_ms);
    diagnostics.count_n("intersect.pieces", imprint.pieces.len() as u64);
    diagnostics.count_n("intersect.vertices", imprint.vertices.len() as u64);
    diagnostics.count_n("intersect.edge_splits", imprint.edge_splits.len() as u64);
    let (march_swap_rescues, clip_swap_rescues) = crate::imprint::take_rescue_firings();
    diagnostics.count_n("intersect.march_swap_rescue", march_swap_rescues);
    diagnostics.count_n("intersect.clip_swap_rescue", clip_swap_rescues);
    let touch = TouchCensus::build([first, second], &imprint);
    touch.report(&mut diagnostics);
    let stage_started = Instant::now();
    let (split_first, split_map_first) = apply_edge_splits_with_map(first, 0, &imprint)?;
    let (split_second, split_map_second) = apply_edge_splits_with_map(second, 1, &imprint)?;
    let stage_ms = stage_started.elapsed().as_secs_f64() * 1_000.0;
    profile_stage("edge_split_ms", stage_ms);
    diagnostics.measure_max("timing.edge_split_ms", stage_ms);
    let stage_started = Instant::now();
    crate::classification::classify_profile_begin();
    let fragments_a = crate::fragment::fragment_solid_against(&split_first, 0, &imprint, crate::fragment::bounded_carriers(second).then_some((bounds[1], policy.sew_search)))?;
    let fragments_b = crate::fragment::fragment_solid_against(&split_second, 1, &imprint, crate::fragment::bounded_carriers(first).then_some((bounds[0], policy.sew_search)))?;
    crate::classification::classify_profile_report("fragment");
    let stage_ms = stage_started.elapsed().as_secs_f64() * 1_000.0;
    profile_stage("fragment_ms", stage_ms);
    diagnostics.measure_max("timing.fragment_ms", stage_ms);
    diagnostics.count_n(
        "fragment.candidates",
        (fragments_a.len() + fragments_b.len()) as u64,
    );
    touch.report_fragments(&[&fragments_a, &fragments_b], &mut diagnostics);
    // Operand surface samples for the LEGITIMATE-EMPTY adjudication below:
    // every fragment test point is a 3D point ON its operand's boundary and
    // inside its face trim, so they witness where the operands' materials sit
    // relative to each other without extra geometry work.
    let a_surface_samples: Vec<Vec3> = fragments_a
        .iter()
        .flat_map(|fragment| {
            std::iter::once(fragment.test_point).chain(fragment.extra_test_points.iter().copied())
        })
        .collect();
    let b_surface_samples: Vec<Vec3> = fragments_b
        .iter()
        .flat_map(|fragment| {
            std::iter::once(fragment.test_point).chain(fragment.extra_test_points.iter().copied())
        })
        .collect();
    let stage_started = Instant::now();
    let mut barrier_edges: HashSet<(u8, u64)> = imprint
        .pieces
        .iter()
        .filter_map(|piece| piece.shared_edge.map(|(operand, edge_id, _)| (operand, edge_id)))
        .collect();
    barrier_edges.extend(imprint.barrier_edges.iter().copied());
    // The barrier set is keyed on ORIGINAL edge ids, but a barrier edge the
    // imprint also SPLIT (rotated equator-tangency: the overlapped cap ring
    // gains a junction at the sphere-seam crossing) reaches fragment
    // selection as its minted sub-edge ids. Remap the barrier through the
    // split ledger — keeping the originals too for unsplit references.
    for (operand, split_map) in [(0u8, &split_map_first), (1u8, &split_map_second)] {
        let minted: Vec<(u8, u64)> = barrier_edges
            .iter()
            .filter(|(barrier_operand, _)| *barrier_operand == operand)
            .filter_map(|(_, edge_id)| split_map.get(edge_id))
            .flat_map(|sub_ids| sub_ids.iter().map(|sub_id| (operand, *sub_id)))
            .collect();
        barrier_edges.extend(minted);
    }
    crate::classification::classify_profile_begin();
    let coincident = coincident_pairs(
        &fragments_a,
        &fragments_b,
        first,
        second,
        &imprint.cosurface_pairs,
        tolerance,
    )?;
    let sector_mode = sector::sector_mode(options);
    let mut select_census = sector::SelectCensus::default();
    let mut selected = select_fragments(
        fragments_a,
        fragments_b,
        first,
        second,
        operation,
        tolerance,
        &barrier_edges,
        &imprint,
        &coincident,
        sector_mode,
        &mut select_census,
    )?;
    let resewn = resew_coincident_rims(
        &mut selected,
        [&split_first, &split_second],
        [first, second],
        &imprint.cosurface_pairs,
        operation,
        tolerance,
    )?;
    diagnostics.count_n("select.coincident_pairs", coincident.len() as u64);
    diagnostics.count_n("select.rims_resewn", resewn as u64);
    crate::classification::classify_profile_report("select");
    let stage_ms = stage_started.elapsed().as_secs_f64() * 1_000.0;
    profile_stage("select_ms", stage_ms);
    diagnostics.measure_max("timing.select_ms", stage_ms);
    diagnostics.count_n("select.fragments", selected.len() as u64);
    // One census line per selection as well.
    diagnostics.count_n("select.sector_eligible", select_census.eligible);
    diagnostics.count_n("select.sector_primary", select_census.sector_primary);
    diagnostics.count_n("select.sector_fallback", select_census.sector_fallback);
    diagnostics.count_n("select.sector_disagree", select_census.disagree);
    diagnostics.count_n("select.ray_casts", select_census.ray_casts);
    diagnostics.count_n("select.on_votes", select_census.on_votes);
    diagnostics.count_n("select.overturns", select_census.overturns);
    if sector_mode != sector::SectorMode::Off || std::env::var_os("BREP_SECTOR_CENSUS").is_some() {
        sector::sector_census_line(|| {
            format!(
                "selection mode={sector_mode:?} select_ms={stage_ms:.3} selected={} eligible={} primary={} fallback={} disagree={} ray_casts={} on_votes={} overturns={}",
                selected.len(),
                select_census.eligible,
                select_census.sector_primary,
                select_census.sector_fallback,
                select_census.disagree,
                select_census.ray_casts,
                select_census.on_votes,
                select_census.overturns
            )
        });
    }
    // LEGITIMATE-EMPTY RESULTS: an intersect of DISJOINT operands (or a
    // subtract whose left operand is entirely CONSUMED by the right) selects
    // zero fragments, and assembly used to refuse with "operation produced no
    // boundary faces" — 384 of the 474 post-wrapped-band pool errors were
    // this, not bugs. Empty is only blessed on TWO independent proofs:
    //   (a) the imprint recorded NO section evidence (no accepted pierce
    //       seed, no traced SSI branch, no minted piece) — a silently-lost
    //       section (the trial-489 class) always leaves upstream evidence
    //       even when every downstream sub-segment is clipped away, while
    //       pure material disjointness leaves none (verified: 489's
    //       recreated silent-miss state keeps erroring; per-face surface
    //       samples ALONE missed its 235 mm³ pocket, which is why (a) is
    //       required and sampling alone was rejected);
    //   (b) the operands' surface samples agree (every fragment test point
    //       classified against the other solid):
    //       intersect — no sample of either operand strictly inside the
    //       other; subtract — no left-operand sample strictly outside the
    //       right.
    // Contact/graze pairs (evidence exists, material still disjoint) stay
    // errors — conservative by design. Escape hatch: BREP_EMPTY_BOOLEAN=0
    // restores the unconditional error.
    if selected.is_empty()
        && !imprint.section_evidence
        && std::env::var("BREP_EMPTY_BOOLEAN").as_deref() != Ok("0")
    {
        let strictly_inside = |samples: &[Vec3], other: &BrepSolid| -> Result<bool, KernelRefusal> {
            for &sample in samples {
                if classify_point(sample, other, tolerance).or_refuse(KernelStage::Validate, "classify_point")?.class == PointClass::In {
                    return Ok(true);
                }
            }
            Ok(false)
        };
        let outside_witness = |samples: &[Vec3], other: &BrepSolid| -> Result<bool, KernelRefusal> {
            for &sample in samples {
                if classify_point(sample, other, tolerance).or_refuse(KernelStage::Validate, "classify_point")?.class == PointClass::Out {
                    return Ok(true);
                }
            }
            Ok(false)
        };
        let legitimate = match operation {
            BooleanOperation::Intersect => {
                !strictly_inside(&a_surface_samples, second)?
                    && !strictly_inside(&b_surface_samples, first)?
            }
            BooleanOperation::Subtract => !outside_witness(&a_surface_samples, second)?,
            BooleanOperation::Union => false,
        };
        if legitimate {
            diagnostics.event(
                DiagnosticSeverity::Info,
                KernelStage::Select,
                "boolean.empty_result",
                format!("{operation:?} of witnessed-non-overlapping operands is empty"),
            );
            fit_scope.close().report_into(&mut diagnostics);
            polyline_scope.close().report_into(&mut diagnostics);
            diagnostics.measure_max(
                "timing.total_ms",
                operation_started.elapsed().as_secs_f64() * 1_000.0,
            );
            return Ok((
                KernelOutcome {
                    value: empty_solid(),
                    diagnostics,
                },
                None,
            ));
        }
    }
    // TOUCHING-EMPTY RESULTS (`BREP_TOUCHING_EMPTY`): operands that only
    // touch (faces in contact, a flush pocket's tool inside its block) leave
    // section evidence, so the legitimate-empty proof above does not apply,
    // and the regularized intersection or difference is empty all the same.
    // Empty is blessed here on two independent proofs: the same surface-sample
    // witnesses as above, and inclusion-exclusion over another exact-lane
    // boolean of the same pair, which must put this one's volume at zero to
    // a billionth of the larger operand's.
    if selected.is_empty()
        && imprint.section_evidence
        && !matches!(operation, BooleanOperation::Union)
        && touching_empty_enabled()
        && !TOUCHING_PROOF.with(std::cell::Cell::get)
    {
        let strict = |samples: &[Vec3], other: &BrepSolid, class: PointClass| -> Result<bool, KernelRefusal> {
            for &sample in samples {
                if classify_point(sample, other, tolerance).or_refuse(KernelStage::Validate, "classify_point")?.class == class {
                    return Ok(true);
                }
            }
            Ok(false)
        };
        let witnessed = match operation {
            BooleanOperation::Intersect => {
                !strict(&a_surface_samples, second, PointClass::In)? && !strict(&b_surface_samples, first, PointClass::In)?
            }
            BooleanOperation::Subtract => !strict(&a_surface_samples, second, PointClass::Out)?,
            BooleanOperation::Union => false,
        };
        // Two cylinders, or two balls, can meet in a lens a hair deep
        // (parallel cylinders 1.8e-6 into each other) that the
        // imprint misses and every other boolean of the pair also reads as a
        // touch. Their depth is closed form; one beyond the model tolerance
        // is a real body, which is not empty.
        let overlap_free = !hair_deep_overlap(first, second, tolerance);
        if witnessed && overlap_free {
            TOUCHING_PROOF.with(|flag| flag.set(true));
            let proof = inclusion_exclusion_volume(first, second, operation, options, body_volume(first), body_volume(second), false);
            TOUCHING_PROOF.with(|flag| flag.set(false));
            // Touching operands read zero to rounding (about 1e-13 here); a
            // band in model tolerances over the area would also bless a
            // genuine overlap by a hair (parallel cylinders a few microns
            // deep), so it is a billionth of the larger operand's volume.
            let volume = |solid: &BrepSolid| body_volume(solid).unwrap_or(f64::INFINITY);
            let band = 1e-9 * volume(first).max(volume(second));
            if let Some(proof) = proof.filter(|proof| proof.expected.is_finite() && proof.expected.abs() <= band) {
                diagnostics.event(
                    DiagnosticSeverity::Info,
                    KernelStage::Select,
                    "boolean.empty_touching",
                    format!(
                        "{operation:?} of touching operands is empty: {} reads {:.3e} by {} (band {band:.3e})",
                        proof.formula, proof.expected, proof.reading
                    ),
                );
                fit_scope.close().report_into(&mut diagnostics);
                polyline_scope.close().report_into(&mut diagnostics);
                diagnostics.measure_max("timing.total_ms", operation_started.elapsed().as_secs_f64() * 1_000.0);
                return Ok((KernelOutcome { value: empty_solid(), diagnostics }, None));
            }
        }
    }
    let solids = [(0, &split_first), (1, &split_second)]
        .into_iter()
        .collect::<HashMap<_, _>>();
    let dump = BooleanStageDump::open();
    if let Some(dump) = &dump {
        dump.write("first", &split_first);
        dump.write("second", &split_second);
        dump.write("selected", &selected);
    }
    let stage_started = Instant::now();
    // Result face `k` is minted from `selected[k - 1]` (one face per fragment,
    // ids from 1 in order): the map the tangent-section attribution reads a
    // refusal's face ids through.
    let assembled_origins: Vec<FaceKey> = if imprint.tangent_sections.is_empty() {
        Vec::new()
    } else {
        selected
            .iter()
            .map(|fragment| FaceKey { operand: fragment.operand, face_id: fragment.source_face_id })
            .collect()
    };
    // A record the grazing-pierce path minted (`TangentSectionRecord::
    // grazing` non-empty, whether or not the lane then found a contact to
    // name) yields to a tangent NODE the imprint classified: the node is the
    // nearer cause and `attribute_to_tangent_node` names it on the way out
    // (the G1 elbow over its own-radius arm: the arm's cap circle also
    // pierces the elbow under the seed gate, and the lane sees the join).
    let attributable: Vec<crate::imprint::TangentSectionRecord> = imprint
        .tangent_sections
        .iter()
        .filter(|section| imprint.tangent_nodes.is_empty() || section.grazing.is_empty())
        .cloned()
        .collect();
    let attribute = |error: KernelRefusal| {
        attribute_to_tangent_section(error, &attributable, &assembled_origins)
    };
    let cellular_result = matches!(operation, BooleanOperation::Union)
        && cellular::enabled(options)
        && cellular::complete_contact(first, second, &selected, tolerance);
    let (solid, scope) = if cellular_result {
        (assemble::assemble_cellular_fragments(selected, &solids, &imprint, tolerance)
            .map_err(attribute)?, None)
    } else if untouched_passthrough(options) {
        assemble_fragments_scoped(selected, &solids, &imprint, tolerance, &touch, touched_assembly(options))
            .map_err(attribute)?
    } else {
        (assemble_fragments(selected, &solids, &imprint, tolerance).map_err(attribute)?, None)
    };
    if let Some(scope) = &scope {
        diagnostics.count_n("passthrough.faces_examined", scope.examined.len() as u64);
    }
    if let Some(dump) = &dump {
        dump.write("assembled", &solid);
    }
    let stage_ms = stage_started.elapsed().as_secs_f64() * 1_000.0;
    profile_stage("assemble_ms", stage_ms);
    diagnostics.measure_max("timing.assemble_ms", stage_ms);
    let stage_started = Instant::now();
    let declines_before = crate::face_merge::declines_served();
    let area_checks_before = crate::face_merge::boundary_area_checks();
    if std::env::var("BREP_DEBUG_PREMERGE_VALIDATE").is_ok() {
        let issues = solid.validate();
        eprintln!(
            "pre-merge validation: {} issue(s){}",
            issues.len(),
            issues
                .first()
                .map(|issue| format!(" — first: {}", issue.message))
                .unwrap_or_default()
        );
    }
    let solid = if solid.kind != crate::BodyKind::Cellular && options.merge_coplanar_faces
        && std::env::var("BREP_COPLANAR_MERGE").as_deref() != Ok("0")
    {
        let merged = match &scope {
            // Every pair merges as before (an operand's own coplanar
            // neighbours included); the closing validation reads the
            // examined faces and the merged ones.
            Some(scope) => merge_same_surface_faces_scoped(
                &solid,
                tolerance,
                &options.keep_unmerged_name_substrs,
                &scope.examined,
            )?,
            None => merge_same_surface_faces_excluding(
                &solid,
                tolerance,
                &options.keep_unmerged_name_substrs,
            )?,
        };
        // Face merging can make previously separate collinear boundary
        // segments incident to the same pair of faces. Run continuation
        // cleanup again, matching the post-merge normalization performed by
        // the former assembly pipeline.
        let merged = merge_curve_continuation_edges(&merged, tolerance).map_err(|refusal| KernelRefusal { stage: KernelStage::Validate, ..refusal })?;
        rejoin::rejoin_unless_referenced(merged, options, tolerance)
    } else {
        solid
    };
    if let Some(dump) = &dump {
        dump.write("merged", &solid);
    }
    let stage_ms = stage_started.elapsed().as_secs_f64() * 1_000.0;
    profile_stage("face_merge_ms", stage_ms);
    if crate::profile::profile_enabled() {
        profile_line!("boolean.merge_declines served={}", crate::face_merge::declines_served() - declines_before);
        let area_checks = crate::face_merge::boundary_area_checks();
        if area_checks != area_checks_before {
            profile_line!("boolean.merge_area_checks boundary={} integrated={}", area_checks.0 - area_checks_before.0, area_checks.1 - area_checks_before.1);
        }
    }
    diagnostics.measure_max("timing.face_merge_ms", stage_ms);
    let validation_started = Instant::now();
    let validation = solid.validate_detailed_scoped(&policy, scope.as_ref().map(|scope| &scope.examined));
    profile_stage("final_validate_ms", validation_started.elapsed().as_secs_f64() * 1000.0);
    diagnostics.count_n("validate.issues", validation.issues.len() as u64);
    diagnostics.count_n(
        "validate.wire_warnings",
        validation.wire_warnings.len() as u64,
    );
    diagnostics.measure_max("validate.max_pcurve_error", validation.max_pcurve_error);
    for warning in &validation.wire_warnings {
        diagnostics.event(
            DiagnosticSeverity::Warning,
            KernelStage::Validate,
            "validate.uv_wire",
            warning.message.clone(),
        );
    }
    for issue in &validation.issues {
        diagnostics.event(
            DiagnosticSeverity::Error,
            KernelStage::Validate,
            "validate.brep",
            issue.message.clone(),
        );
    }
    fit_scope.close().report_into(&mut diagnostics);
    polyline_scope.close().report_into(&mut diagnostics);
    diagnostics.measure_max(
        "timing.total_ms",
        operation_started.elapsed().as_secs_f64() * 1_000.0,
    );
    if !validation.issues.is_empty() {
        return Err(KernelRefusal::new(
            RefusalClass::InvalidResultTopology { issues: validation.issues.len() as u32 },
            KernelStage::Validate,
            format!(
            "boolean_operation: invalid result: {:?}",
            validation.issues
        )));
    }
    // Optional semantic oracle (off by default, no perf hit): under
    // BREP_DEBUG_BOOL, cross-check the well-formed result against the CSG
    // point-membership expectation and scan for residual coincident-but-unwelded
    // geometry. Purely diagnostic and NON-rejecting — a statistical check must
    // never fail a valid op (see oracle.rs) — so it only ever warns.
    if std::env::var("BREP_DEBUG_BOOL").is_ok() {
        if let Ok(report) =
            crate::oracle::boolean_semantic_disagreement(first, second, operation, &solid, 200)
        {
            if report.is_flagged() {
                eprintln!(
                    "[oracle] semantic disagreement rate {:.4} ({} of {} decidable points); sample {:?}",
                    report.disagreement_rate,
                    report.disagreements.len(),
                    report.considered,
                    report.sample_disagreement()
                );
            }
        }
        let fusable_band = crate::tolerance::assembler_weld(policy.model);
        let fusables = crate::oracle::boolean_residual_fusables(&solid, fusable_band);
        if !fusables.is_empty() {
            eprintln!(
                "[oracle] {} residual fusable(s) after weld; sample {:?}",
                fusables.len(),
                fusables.first()
            );
        }
    }
    let candidates = scope.as_ref().map(|scope| {
        let unproved = (scope_proved_apart_on() && plain_hull_solid(first) && plain_hull_solid(second))
            .then(|| imprint.faces_in_unproved_pairs.as_ref())
            .flatten()
            .map(|faces| faces.iter().copied().collect::<HashSet<_>>());
        let candidates = soundness_candidates(
            &solid,
            scope,
            [operand_bounds(first), operand_bounds(second)],
            policy.sew_search,
            unproved.as_ref(),
        );
        diagnostics.count_n("passthrough.soundness_candidates", candidates.len() as u64);
        crate::soundness::census::profile_operation(&solid, &[first, second], Some(&scope.touched), "boolean");
        candidates
    });
    Ok((
        KernelOutcome {
            value: solid,
            diagnostics,
        },
        candidates,
    ))
}


