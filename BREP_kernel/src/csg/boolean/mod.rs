use crate::arrangement::Vec2;
use crate::classification::{classify_point, parameter_point_in_face, PolygonClass};
use crate::fragment::{FaceFragmentRecord, FragmentEdgeSource};
use crate::imprint::{
    EdgeSplitRecord, FaceImprints, FaceKey, ImprintOptions, ImprintPieceRecord,
    ImprintResultRecord, ImprintVertex,
};
use crate::tolerance::{assembler_weld, commit_weld};
use crate::topology::{
    adaptive_coedge_error, BrepSolid, CoedgeRecord, EdgeRecord, FaceRecord, LoopRecord,
    ShellRecord, VertexRecord,
};
use crate::{
    KernelRefusal, OrRefuse, RefusalClass,
    apply_edge_splits, apply_edge_splits_with_map, build_imprints, build_pcurve_on_surface,
    build_pcurve_on_surface_range, classify_surface_pair,
    fragment_solid, interpolate_curve, merge_curve_continuation_edges,
    merge_same_surface_faces_excluding,
    project_point_to_curve, project_point_to_surface, solid_signed_volume, AffineTransform,
    DiagnosticSeverity, KernelDiagnostics, KernelOutcome, KernelStage, KernelTolerances, NurbsCurve,
    PointClass, SolidClassifier, SurfacePairRelation, Vec3,
};
use rustc_hash::{FxHashMap as HashMap, FxHashSet as HashSet};
use serde::{Deserialize, Serialize};
use web_time::Instant;

thread_local! {
    /// Running count (per thread) of one-use edges the EDGE-CONFORMANCE
    /// repair lanes merged or bridged.  The lanes only fire on an assembly
    /// that would otherwise refuse, so a zero delta across an operation is
    /// an honest witness that the boolean assembled cleanly without the
    /// repair.  Test-observable; not part of any result.
    pub(crate) static CONFORMANCE_REPAIRS: std::cell::Cell<u64> =
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
    /// merge (only consulted when [`merge_coplanar_faces`] is on). A face whose
    /// `name` CONTAINS any of these is emitted unchanged instead of being
    /// coalesced with a mergeable neighbour; faces matching none merge exactly as
    /// before. Default empty — no caller sees a behavior change unless it opts
    /// in. Sheet metal uses this to keep every thickness/side wall face per
    /// outline segment in the folded solid (so a flange can still attach to a
    /// specific segment: fusing two collinear thickness faces would erase the
    /// segment boundary).
    #[serde(default)]
    pub keep_unmerged_name_substrs: Vec<String>,
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
        }
    }
}

mod select;
mod assemble;
mod operand_state;
mod rim;
mod coincident;

use rim::*;
use select::*;
use coincident::*;
pub(crate) use assemble::{
    assemble_fragments, assemble_open_fragments, commit_nearby_edge_endpoints,
    edge_interior_lies_on, finalize_assembled_solid,
};
// Currently referenced only inside `assemble`, so the re-export is unused.
#[allow(unused_imports)]
pub(crate) use assemble::apply_assembly_heal_chain;

pub fn boolean_operation(
    first: &BrepSolid,
    second: &BrepSolid,
    operation: BooleanOperation,
    options: &BooleanOptions,
) -> Result<BrepSolid, KernelRefusal> {
    match boolean_operation_with_diagnostics(first, second, operation, options) {
        Ok(outcome) => Ok(outcome.value),
        Err(error) => {
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
fn subtract_pinches_at_internal_tangency(
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
    let scale = crate::tolerance::solid_scale(first)
        .max(crate::tolerance::solid_scale(second))
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
const PERTURBATION_FRACTIONS: [f64; 5] = [3e-5, 1e-4, 3e-4, 1e-3, 3e-3];

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
struct MirrorTrend {
    /// The last rung refused by its mirror: its offset and signed error.
    last: Option<(f64, f64)>,
    closed: bool,
}

impl MirrorTrend {
    /// Records a rung refused by its mirror at `offset` with signed error
    /// `error`. When it and the direction's last refused rung close the
    /// direction, and `close` allows it, the direction is closed and the
    /// earlier rung's (offset, signed error) returned.
    fn refused(&mut self, offset: f64, error: f64, close: bool) -> Option<(f64, f64)> {
        let earlier = self
            .last
            .filter(|&(_, earlier)| earlier.signum() == error.signum() && error.abs() >= earlier.abs())
            .filter(|_| close);
        self.last = Some((offset, error));
        self.closed |= earlier.is_some();
        earlier
    }

    /// A rung whose mirror clears: the direction's record starts again.
    fn cleared(&mut self) {
        self.last = None;
    }
}

/// Non-degenerate edges a solid's faces use exactly once.
fn one_use_edges(solid: &BrepSolid) -> usize {
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
        vertices,
        pieces,
        tangent_nodes: Vec::new(),
        barrier_edges: Vec::new(),
        section_evidence,
        cosurface_pairs: Vec::new(),
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
    if operands.is_empty() {
        return Err(KernelRefusal::input(KernelStage::Collect, "operands", "boolean_operation_nary: no operands provided"));
    }
    if operands.len() == 1 {
        return Ok(operands[0].clone());
    }
    if operands.len() > 255 {
        return Err(KernelRefusal::input(KernelStage::Collect, "operands", "boolean_operation_nary: at most 255 operands supported"));
    }
    let debug = std::env::var("BREP_DEBUG_BOOL").is_ok();
    let options = BooleanOptions::default();

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
        crate::heal::refit_inherited_trims(operand, &policy)?;
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
        merge_curve_continuation_edges(&merged, tolerance).map_err(|refusal| KernelRefusal { stage: KernelStage::Validate, ..refusal })?
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
/// refusal that could never take the ladder still cannot.
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

pub fn boolean_operation_with_diagnostics(
    first: &BrepSolid,
    second: &BrepSolid,
    operation: BooleanOperation,
    options: &BooleanOptions,
) -> Result<KernelOutcome<BrepSolid>, KernelRefusal> {
    let mut tangent_nodes = Vec::new();
    let mut outcome = boolean_pipeline(first, second, operation, options, &mut tangent_nodes)
        .map_err(|error| attribute_to_tangent_node(error, &tangent_nodes))?;
    // The assembly's single exit. A boolean can hand back a body whose walls
    // pass through each other while `validate()` is silent and the volume
    // measures right, which is exactly the failure a regularized boolean must
    // not report as success. A crossing the OPERANDS already had is theirs and
    // passes through — the defect came in with the input and refusing it here
    // would blame the wrong operation — so the operands are only scanned when
    // the result is flagged, and the common case pays for one scan.
    // The acceptance's refusal is typed at its origin (an `UnsoundResult`
    // naming the defect and the faces); it was classed `Internal` here while
    // `accept_sound_against` still returned text.
    outcome.value = crate::accept_sound_against(&[first, second], outcome.value, "boolean")?;
    Ok(outcome)
}

/// Stage wall-clock line on stderr when `BREP_PROFILE` is set — the same switch
/// `imprint.profile` and `assemble.*_ms` use — so a REFUSED boolean, whose
/// `KernelDiagnostics` never reach a caller, still shows where its time went.
fn profile_stage(code: &str, ms: f64) {
    if std::env::var("BREP_PROFILE").is_ok() {
        eprintln!("boolean.{code}={ms:.2}");
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
fn boolean_policy(
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
    // Then refit, on each operand's own carriers, any inherited trim off the
    // pcurve construction contract whose edge lies on every incident carrier
    // (`heal::refit_inherited_trims`; a byte-identical no-op without one).
    crate::heal::refit_inherited_trims(&mut first_owned, policy)?;
    crate::heal::refit_inherited_trims(&mut second_owned, policy)?;
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
    let policy = boolean_policy(first, second, options)?;
    let (first, second) = prepared_operands(first, second, &policy)?;
    let imprint = build_imprints(&first, &second, &boolean_imprint_options(options, policy.model))?;
    let (split_first, _) = apply_edge_splits_with_map(&first, 0, &imprint)?;
    let (split_second, _) = apply_edge_splits_with_map(&second, 1, &imprint)?;
    Ok((split_first, split_second))
}

fn boolean_pipeline(
    first: &BrepSolid,
    second: &BrepSolid,
    operation: BooleanOperation,
    options: &BooleanOptions,
    tangent_nodes: &mut Vec<Vec3>,
) -> Result<KernelOutcome<BrepSolid>, KernelRefusal> {
    let replaced_operands = operand_state::replace_operands(first, second, operation, options);
    let (first, second) = match &replaced_operands {
        Some((first, second)) => (first, second),
        None => (first, second),
    };
    let policy = boolean_policy(first, second, options)?;
    let tolerance = policy.model;
    let (first_owned, second_owned) = prepared_operands(first, second, &policy)?;
    let first = &first_owned;
    let second = &second_owned;

    let mut diagnostics = KernelDiagnostics::default();
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
    let stage_started = Instant::now();
    let (split_first, split_map_first) = apply_edge_splits_with_map(first, 0, &imprint)?;
    let (split_second, split_map_second) = apply_edge_splits_with_map(second, 1, &imprint)?;
    let stage_ms = stage_started.elapsed().as_secs_f64() * 1_000.0;
    profile_stage("edge_split_ms", stage_ms);
    diagnostics.measure_max("timing.edge_split_ms", stage_ms);
    let stage_started = Instant::now();
    crate::classification::classify_profile_begin();
    let fragments_a = fragment_solid(&split_first, 0, &imprint)?;
    let fragments_b = fragment_solid(&split_second, 1, &imprint)?;
    crate::classification::classify_profile_report("fragment");
    let stage_ms = stage_started.elapsed().as_secs_f64() * 1_000.0;
    profile_stage("fragment_ms", stage_ms);
    diagnostics.measure_max("timing.fragment_ms", stage_ms);
    diagnostics.count_n(
        "fragment.candidates",
        (fragments_a.len() + fragments_b.len()) as u64,
    );
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
    let mut selected = select_fragments(
        fragments_a,
        fragments_b,
        first,
        second,
        operation,
        tolerance,
        &barrier_edges,
        &imprint.cosurface_pairs,
        &coincident,
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
            return Ok(KernelOutcome {
                value: BrepSolid {
                    mass_properties_cache: Default::default(),
                    id: 0,
                    vertices: Vec::new(),
                    edges: Vec::new(),
                    shells: Vec::new(),
                    genus: 0,
                },
                diagnostics,
            });
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
    let solid = assemble_fragments(selected, &solids, &imprint, tolerance)?;
    if let Some(dump) = &dump {
        dump.write("assembled", &solid);
    }
    let stage_ms = stage_started.elapsed().as_secs_f64() * 1_000.0;
    profile_stage("assemble_ms", stage_ms);
    diagnostics.measure_max("timing.assemble_ms", stage_ms);
    let stage_started = Instant::now();
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
    let solid = if options.merge_coplanar_faces
        && std::env::var("BREP_COPLANAR_MERGE").as_deref() != Ok("0")
    {
        let merged = merge_same_surface_faces_excluding(
            &solid,
            tolerance,
            &options.keep_unmerged_name_substrs,
        )?;
        // Face merging can make previously separate collinear boundary
        // segments incident to the same pair of faces. Run continuation
        // cleanup again, matching the post-merge normalization performed by
        // the former assembly pipeline.
        merge_curve_continuation_edges(&merged, tolerance).map_err(|refusal| KernelRefusal { stage: KernelStage::Validate, ..refusal })?
    } else {
        solid
    };
    if let Some(dump) = &dump {
        dump.write("merged", &solid);
    }
    let stage_ms = stage_started.elapsed().as_secs_f64() * 1_000.0;
    profile_stage("face_merge_ms", stage_ms);
    diagnostics.measure_max("timing.face_merge_ms", stage_ms);
    let validation = solid.validate_detailed(&policy);
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
    Ok(KernelOutcome {
        value: solid,
        diagnostics,
    })
}

