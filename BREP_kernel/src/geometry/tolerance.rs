//! # Tolerance taxonomy — which bands may scale with part size, and which must not
//!
//! Every numeric tolerance in this kernel belongs to exactly one of the KINDS
//! below.  The distinction that matters for correctness is whether a band may be
//! **size-coupled** (widened in proportion to the local part extent) or must stay
//! a **tight absolute** floor.  Getting this wrong is not academic: coupling the
//! base spatial `model` tolerance to part size was tried and REJECTED because
//! `model` also feeds the fit-accuracy fields, and scaling it degraded SSI curve
//! fitting enough to produce invalid boolean topology (see the B1 finding in
//! `revolve_pole_union_fixture`).  The rules:
//!
//! * **Identity / coincidence** — "are these two entities the same point / are
//!   these surfaces the same surface?".  MAY size-couple: a big part legitimately
//!   needs a proportionally wider identity band.  Couple it *at the site* via
//!   [`KernelTolerances::heal_band`] (`max(model, diagonal * k)`), taking the
//!   local `diagonal`/extent explicitly — never by inflating the global `model`.
//!
//! * **Fit-accuracy** — `intersection_fit`, the SSI/CSI marcher, curve/surface
//!   fitting.  MUST stay TIGHT.  Do NOT size-couple: this is "how closely must
//!   committed geometry agree?", and loosening it silently degrades every export.
//!   This is the field that broke `revolve_pole_union` when coupled (B1).
//!
//! * **Weld / sew** — the distinct-vertex / weld radius used when knitting
//!   endpoints and edges together (assembler weld, endpoint commit, vertex
//!   merge).  A *search* radius answering "which entities might match?", floored
//!   to a small absolute so noise-free models still weld; see the named weld
//!   accessors on [`KernelTolerances`].
//!
//! * **Knot-parameter** — knot-vector identity and numerical knot dedup.  Two
//!   distinct purposes at two distinct values: knot IDENTITY
//!   (`KNOT_IDENTITY_TOL`, 1e-9) and numerical knot DEDUP (`KNOT_DEDUP_EPS`,
//!   1e-12), both single-sourced in `curve.rs`.  Parameter-space, not spatial;
//!   derived from a spatial band only via [`parametric_tolerance`].
//!
//! * **Angular** — direction/normal agreement in radians (`angular`).  A fixed
//!   absolute; does not scale with part size.
//!
//! * **Strict-interior** — small skip epsilons that keep a parameter off the
//!   exact domain end (so knot insertion / split does not refuse it).  Fixed,
//!   derived from the knot-identity band, not size-coupled.
//!
//! * **Floating-point floor** — the `1e-12`/`1e-15` guards that keep a division
//!   or a degenerate direction from blowing up.  Never a modelling tolerance.
//!
//! Bottom line: identity/weld bands couple to size *locally* through
//! [`KernelTolerances::heal_band`]/[`solid_scale`]; fit-accuracy, angular, and
//! the floating-point floors stay tight.

use crate::topology::BrepSolid;
use serde::{Deserialize, Serialize};

/// Ordered accuracy targets plus deliberately separate geometric search radii.
///
/// Search tolerances answer "which entities might match?".  Accuracy
/// tolerances answer "how closely must committed geometry agree?".  Keeping
/// those questions separate prevents a generous sewing search radius from
/// becoming the accuracy of the exported BREP.
#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Serialize)]
#[serde(default)]
pub struct KernelTolerances {
    /// Iterative projection/refinement convergence target.
    pub convergence: f64,
    /// Vertex identity and exact topological coincidence tolerance.
    pub model: f64,
    /// Maximum geometric error accepted while fitting SSI curves.
    pub intersection_fit: f64,
    /// Maximum edge/pcurve/carrier disagreement in a valid in-memory BREP.
    pub pcurve_consistency: f64,
    /// Output edge-to-carrier contract used before STEP serialization.
    pub export_knit: f64,
    /// Candidate radius used while looking for endpoints or edges to weld.
    pub sew_search: f64,
    /// Features shorter than this are candidates for explicit sliver repair.
    pub sliver: f64,
    /// Angular tolerance in radians.
    pub angular: f64,
}

impl Default for KernelTolerances {
    fn default() -> Self {
        Self::for_scale(1.0, 1e-7)
    }
}

impl KernelTolerances {
    /// Construct a scale-aware policy while preserving the historical model
    /// identity tolerance used by the public API.
    ///
    /// The base spatial identity tolerance `model` is deliberately NOT coupled
    /// to part size here: coupling it was tried and rejected because `model`
    /// also feeds fit-accuracy fields (`intersection_fit`, `sew_search`), so
    /// scaling it degrades SSI curve fitting on ordinary parts and produces
    /// invalid boolean topology (see `revolve_pole_union_fixture`).  Identity
    /// bands that legitimately scale with size are size-coupled per site via
    /// [`KernelTolerances::heal_band`] instead, leaving fit accuracy tight.
    pub fn for_scale(scale: f64, model: f64) -> Self {
        let scale = scale.abs().max(1.0);
        let model = model.abs().max(1e-12);
        Self {
            convergence: (model * 1e-3).clamp(1e-12, model),
            model,
            intersection_fit: (model * 20.0).max(scale * 1e-8),
            // Existing fitted NURBS intersections can deviate by a few
            // microns.  This remains an explicit contract rather than a
            // validator-local magic number.
            pcurve_consistency: (model * 200.0).max(4e-3),
            export_knit: (model * 100.0).max(4e-3),
            sew_search: (model * 20.0).max(scale * 1e-8),
            sliver: (model * 4.0).max(scale * 1e-10),
            angular: 1e-4,
        }
    }

    pub fn for_solid(solid: &BrepSolid, model: f64) -> Self {
        Self::for_scale(solid_scale(solid), model)
    }

    pub fn for_pair(first: &BrepSolid, second: &BrepSolid, model: f64) -> Self {
        Self::for_scale(solid_scale(first).max(solid_scale(second)), model)
    }

    /// Reject policies whose accuracy ladder is inverted or whose search
    /// radii cannot even find model-identical entities.
    pub fn check(&self) -> Result<(), String> {
        let positive = [
            ("convergence", self.convergence),
            ("model", self.model),
            ("intersection_fit", self.intersection_fit),
            ("pcurve_consistency", self.pcurve_consistency),
            ("export_knit", self.export_knit),
            ("sew_search", self.sew_search),
            ("sliver", self.sliver),
            ("angular", self.angular),
        ];
        for (name, value) in positive {
            if !value.is_finite() || value <= 0.0 {
                return Err(format!("invalid {name} tolerance {value}"));
            }
        }
        for ((first_name, first), (second_name, second)) in [
            (("convergence", self.convergence), ("model", self.model)),
            (
                ("model", self.model),
                ("intersection_fit", self.intersection_fit),
            ),
        ] {
            if first > second {
                return Err(format!(
                    "tolerance ladder violated: {first_name} ({first:.3e}) > \
                     {second_name} ({second:.3e})"
                ));
            }
        }
        if self.sew_search < self.model {
            return Err(format!(
                "sew_search ({:.3e}) is below model tolerance ({:.3e})",
                self.sew_search, self.model
            ));
        }
        Ok(())
    }

    /// Canonical accessor for the single spatial tolerance `x` (Golovanov
    /// §4.13).  `model` is that `x`: the base identity/coincidence band from
    /// which parametric, search, and healing tolerances are derived.  Call
    /// sites should prefer this over reaching for `.model` directly so intent
    /// (the ONE spatial tolerance) reads clearly and later levers have a single
    /// seam to evolve.
    pub fn spatial(&self) -> f64 {
        self.model
    }

    /// Size-coupled healing/identity band: the base spatial tolerance floored
    /// by a fraction `k` of the caller's local `diagonal`.  This is the ONE
    /// charter helper for the `max(model, diagonal * k)` pattern that healing
    /// and identity sites otherwise hand-roll (e.g. classification's
    /// near-coincidence probe `(model * 10).max(diagonal * 1e-7)`), so a
    /// size-relative band is derived from `x` in a single place instead of
    /// re-hardcoded per call site (Golovanov §4.13).  Coupling the band HERE —
    /// at the identity/weld/heal site, taking `diagonal` explicitly — gives big
    /// parts a proportionally wider identity band WITHOUT inflating the global
    /// `model` or the fit-accuracy path (`intersection_fit`, the marcher).
    /// `k` is a dimensionless part-per-diagonal factor; `diagonal` is the local
    /// extent the band should scale with.
    pub fn heal_band(&self, diagonal: f64, k: f64) -> f64 {
        self.model.max(diagonal.abs() * k.abs())
    }

    /// Size-coupled acceptance ceiling for the edge-vs-pcurve COINCIDENCE check
    /// in [`BrepSolid::validate`]: "does this coedge's curve-on-surface, pushed
    /// back to 3D, still trace the same locus as the edge's 3D curve?"
    ///
    /// This is an IDENTITY/coincidence band (are the two representations the
    /// SAME edge?), not a fit-accuracy target, so per this module's taxonomy it
    /// MAY — and, for imported geometry, MUST — size-couple with the part: a
    /// vendor STEP file routinely commits an edge's 3D curve and its face
    /// surface as INDEPENDENT approximations that disagree by a small fraction
    /// of the model, and (like Parasolid/OCC, which absorb the gap in a widened
    /// per-edge tolerance) a coincidence band floored to a tight ABSOLUTE
    /// `pcurve_consistency` wrongly rejects such a shared edge on a sub-unit
    /// part.  The gap is intrinsic to the vendor data (it is the closest-point
    /// residual of the edge against the surface, so no pcurve fit can beat it),
    /// and the edge is SHARED — snapping it onto one face's surface only pushes
    /// it off the neighbour's — so accepting the size-relative gap is the
    /// faithful, non-destructive resolution.
    ///
    /// Coupling lives HERE (taking the model `diagonal` explicitly, floored by
    /// the tight `pcurve_consistency`) exactly like [`KernelTolerances::heal_band`],
    /// so the fit target and the STEP-import edge-reconcile screen — which read
    /// the raw `pcurve_consistency` FIELD — stay tight and are NOT relaxed by
    /// this validator-only ceiling.  `PCURVE_ACCEPTANCE_REL` (2.5% of the model
    /// diagonal) sits above the vendor near-miss this admits (~2% of the
    /// diagonal on ABC 00000041) yet far below the many-percent excursion a
    /// genuinely wrong carrier or branch-jumped pcurve produces, so real breakage
    /// is still refused.
    pub fn pcurve_acceptance(&self, diagonal: f64) -> f64 {
        self.pcurve_consistency
            .max(diagonal.abs() * PCURVE_ACCEPTANCE_REL)
    }
}

/// Fraction of the model bounding-box diagonal used as the size-coupled edge/
/// pcurve coincidence ceiling in [`KernelTolerances::pcurve_acceptance`].
pub const PCURVE_ACCEPTANCE_REL: f64 = 0.025;

/// Absolute floor (mm) of the edge-endpoint-vs-vertex identity band used by
/// [`BrepSolid::validate`]: below this a curve end and its topological vertex
/// ARE the same point (vendor export precision), above it the model is asked to
/// state the meeting exactly.
///
/// Single-sourced because the STEP importer must heal precisely the misses this
/// band refuses: healing less leaves an import that cannot validate, healing
/// more rewrites vendor geometry the kernel already accepts as-is (see
/// `heal_imported_edge_endpoints`).
pub const VERTEX_MATCH_FLOOR: f64 = 5e-3;

/// Derive a parametric tolerance from one spatial tolerance and the local
/// derivative magnitude: `e = x / |c'(t)|`.
///
/// The same spatial error corresponds to different parametric errors on
/// every curve and surface, so parameter-space tolerances must be derived
/// locally from a single spatial precision, never written as fixed
/// parameter-space literals (Golovanov, "Geometric Modeling" §4.13).  The
/// derivative floor guards degenerate directions (poles, collapsed edges)
/// from producing an unbounded band; callers that know their domain span
/// should additionally cap the result to a fraction of it.
pub fn parametric_tolerance(spatial: f64, derivative_magnitude: f64) -> f64 {
    spatial.abs().max(1e-15) / derivative_magnitude.abs().max(1e-9)
}

/// Scalar UV band for surface queries at a point with derivative magnitudes
/// `|r_u|`, `|r_v|`: the conservative bound that contains the anisotropic
/// `(x/|r_u|, x/|r_v|)` box is `x` over the smaller derivative.
pub fn surface_uv_tolerance(spatial: f64, du_magnitude: f64, dv_magnitude: f64) -> f64 {
    parametric_tolerance(spatial, du_magnitude.abs().min(dv_magnitude.abs()))
}

// ---------------------------------------------------------------------------
// Weld / distinct-vertex radii — one named source per drifting expression.
//
// The kernel welds "the same vertex reached by two independent fits" at several
// sites, and the survey found the SAME concept written three different ways with
// three different effective values (drift).  These accessors name each variant
// and RETURN its exact prior value, so the drift is visible and single-sourced
// in ONE place WITHOUT changing behaviour (bit-identical by construction).  A
// future decision to truly unify them becomes a one-line edit here.
// ---------------------------------------------------------------------------

/// Absolute floor for the assembler/imprint weld radius (see [`assembler_weld`]).
pub const WELD_FLOOR: f64 = 1e-5;

/// Absolute floor for the endpoint-commit weld radius (see [`commit_weld`]).
/// Deliberately looser than [`WELD_FLOOR`] (1e-4 vs 1e-5): the commit pass runs
/// on already-healed solids where the surviving endpoint gaps are larger than
/// assembly-time vertex noise.  This difference is intentional and preserved.
pub const COMMIT_WELD_FLOOR: f64 = 1e-4;

/// Assembler / imprint weld radius: the model identity tolerance `model` floored
/// at [`WELD_FLOOR`].  Answers "which independently-fitted endpoints are the
/// SAME vertex?" — a search radius, floored generously so a noise-free model
/// (model ~ 1e-7) still welds coincident endpoints into a topological loop.
/// Used by the boolean assembler (`vertex`/`edge`/triple-junction polish) and
/// imprint's edge weld/limit sites.
pub fn assembler_weld(model: f64) -> f64 {
    model.max(WELD_FLOOR)
}

/// Endpoint-commit weld radius (`commit_nearby_edge_endpoints`, used by fillet /
/// heal edge-gap closing): the caller's SEARCH tolerance floored at
/// [`COMMIT_WELD_FLOOR`].  Same distinct-vertex concept as [`assembler_weld`]
/// but a looser floor by design — see [`COMMIT_WELD_FLOOR`].
pub fn commit_weld(search: f64) -> f64 {
    search.max(COMMIT_WELD_FLOOR)
}

/// Size factor applied to imprint vertex-merge bands: `1 + extent`, where
/// `extent` is the operands' bbox extent ([`solid_scale`]).  This replaces the
/// former `1 + ‖point‖` (distance-from-ORIGIN) coupling, an anti-pattern in
/// which a part far from the origin got a wrongly-inflated merge band — a
/// translation-VARIANCE defect (the same fillet at the origin vs translated
/// far away produced different volumes, and eventually a broken result).
/// Deriving the factor from the operands' bbox extent instead makes the band
/// depend on the part's SIZE, not its position, matching the intent of
/// [`solid_scale`] / [`KernelTolerances::heal_band`], and makes the imprint
/// (hence booleans/fillets) translation-INVARIANT.
pub fn merge_scale(extent: f64) -> f64 {
    1.0 + extent
}

/// Absolute floor for the imprint "do these two surfaces COINCIDE?" distance
/// band, applied under `(tolerance * k).max(COINCIDENCE_DISTANCE_FLOOR)` at each
/// coincidence decision (`coplanar_pair` perpendicular gap, `cosurface_pair`
/// projection gap).  This is the single named source for that floor.
///
/// The survey once found this coincidence question answered with two different
/// values (a `1e-5` floor at some imprint sites vs `1e-6` elsewhere); that value
/// drift was already eliminated upstream — the coincidence distance band is now
/// uniformly floored at `1e-6` — so single-sourcing here is bit-identical and
/// only removes the remaining duplicated literal.  The companion normal-parallel
/// test (`|n_a·n_b|` vs `1 - 1e-6/1e-9`) is a separate ANGULAR criterion and is
/// deliberately NOT folded in here.
pub const COINCIDENCE_DISTANCE_FLOOR: f64 = 1e-6;

// ---------------------------------------------------------------------------
// Model scale — the ONE characteristic length every size-relative tolerance in
// this kernel is measured against.
// ---------------------------------------------------------------------------

/// The kernel's single definition of a model's characteristic length: the
/// **bounding-box diagonal** of a point set.
///
/// Two properties define it, and both are load-bearing:
///
/// * **Size-based.** It answers "how big is *this* geometry?", which is the only
///   question a size-relative tolerance may ask.  Note what "this" means: the
///   answer is the extent of the point set handed in, so the caller chooses the
///   scope by choosing the points.  A local solve should pass its own local
///   geometry (see [`curve_model_scale`]) rather than the whole solid's
///   vertices, or a 2 mm feature inherits a 2 m frame's band.
/// * **Translation-invariant.** Moving the geometry rigidly does not change it,
///   so the identical feature solves to the identical tolerance wherever the
///   modeller happened to place it.
///
/// # Why the origin-distance form is wrong
///
/// The tempting one-liner `points.map(|p| p.length()).fold(1.0, f64::max)` — the
/// maximum distance from the **world origin** — satisfies neither property:
///
/// * It **grows with placement.** A 10 mm part at the origin gets scale ≈ 10; the
///   same part translated to (5000, 0, 0) gets scale ≈ 5000.  Every tolerance
///   derived from it loosens by 500× for a part that did not change shape, so
///   coincidence, weld and convergence bands silently absorb real geometric
///   error the moment a part is placed away from the origin.
/// * It **ignores size.** Two parts whose bounding boxes differ by three orders
///   of magnitude get the same scale if they sit the same distance out.
/// * It is **translation-VARIANT**, so results are not reproducible under a rigid
///   move: the same fillet at the origin and translated far away produced
///   different volumes, and eventually a broken result.  [`merge_scale`] records
///   that exact failure for the imprint vertex-merge band, which was fixed the
///   same way.
///
/// # Degenerate input
///
/// An empty set, a single point, coincident points, or non-finite coordinates
/// all return `1.0`: there is no meaningful extent to scale by, and a zero (or
/// NaN, or infinite) scale would collapse or explode every derived band.
///
/// # Floors are the caller's business
///
/// `model_scale` returns the RAW diagonal for any well-formed input — it is not
/// floored at 1.0.  Sub-unit parts are real (see
/// [`KernelTolerances::pcurve_acceptance`]'s ABC 00000041 case) and flooring
/// would hand a 0.3 mm part a band larger than itself.  Sites that want the
/// historical `>= 1.0` floor take [`solid_scale`], which applies it explicitly.
pub fn model_scale(points: impl IntoIterator<Item = crate::Vec3>) -> f64 {
    let mut low = crate::Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut high = crate::Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    let mut any = false;
    for point in points {
        any = true;
        low.x = low.x.min(point.x);
        low.y = low.y.min(point.y);
        low.z = low.z.min(point.z);
        high.x = high.x.max(point.x);
        high.y = high.y.max(point.y);
        high.z = high.z.max(point.z);
    }
    if !any {
        return 1.0;
    }
    let diagonal = high.sub(low).length();
    if diagonal.is_finite() && diagonal > 0.0 {
        diagonal
    } else {
        1.0
    }
}

/// [`model_scale`] of a solid's vertices — the RAW bounding-box diagonal, with
/// no `>= 1.0` floor.  This is what direct-edit wants: a sub-unit part must keep
/// sub-unit bands.  (`BrepSolid::validate` wants the same thing and still
/// hand-rolls it, for the reason its own comment gives — it answers `0.0`, not
/// `1.0`, for a vertexless solid.  Reconciling that contract is a follow-up, not
/// a free ride-along.)  Use [`solid_scale`] for the floored variant the
/// tolerance policy is built on.
pub fn solid_model_scale(solid: &BrepSolid) -> f64 {
    model_scale(solid.vertices.iter().map(|vertex| vertex.point))
}

/// [`model_scale`] of a curve, sampled uniformly over `[t0, t1]`.
///
/// The characteristic length of a *local* solve — one edge and its immediate
/// neighbourhood — is the extent of the curve being marched, not the extent of
/// the whole solid it belongs to and emphatically not its distance from the
/// origin.  Sampling is uniform and at a fixed count so the result is
/// deterministic and cheap next to the solves it scales.  Probes are `evaluate`,
/// which CLAMPS to the knot domain, so an overshot range still measures the
/// curve's own extent rather than an extrapolation's.
pub fn curve_model_scale(curve: &crate::NurbsCurve, t0: f64, t1: f64) -> Result<f64, String> {
    let mut samples = Vec::with_capacity(CURVE_SCALE_SAMPLES + 1);
    for index in 0..=CURVE_SCALE_SAMPLES {
        let t = t0 + (t1 - t0) * index as f64 / CURVE_SCALE_SAMPLES as f64;
        samples.push(curve.evaluate(t)?);
    }
    Ok(model_scale(samples))
}

/// Uniform sample count used by [`curve_model_scale`].  Enough to bound a
/// closed conic or a wavy spline's extent within a few percent; far too cheap to
/// matter beside the Newton solve whose tolerance it scales.
pub const CURVE_SCALE_SAMPLES: usize = 16;

/// Migration diagnostic for the one-[`model_scale`] change (slice 0).
///
/// Switching a pipeline's `scale` definition moves EVERY tolerance derived from
/// it at once, so the corpus must be measurable before and after.  Call this at
/// a site that is changing over, with the old origin-distance value and the new
/// size-based one, and run the batteries with `BREP_SCALE_DIAG=1` to see which
/// fixtures actually move and by how much.  `ratio > 1` means the old
/// definition was LOOSER than the new one there (the placement-inflation this
/// slice removes); `ratio < 1` means it was tighter.
///
/// Silent and free unless the variable is set: the lookup happens once and the
/// superseded `origin_form` closure is never called otherwise, so the retired
/// definition costs nothing in the shipped path while staying re-measurable.
pub fn report_scale_migration(site: &str, model_form: f64, origin_form: impl FnOnce() -> f64) {
    static ENABLED: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    if !*ENABLED.get_or_init(|| std::env::var("BREP_SCALE_DIAG").is_ok()) {
        return;
    }
    let origin_form = origin_form();
    let ratio = if model_form != 0.0 {
        origin_form / model_form
    } else {
        f64::NAN
    };
    eprintln!("scale-diag site={site} origin={origin_form:.9} model={model_form:.9} ratio={ratio:.6}");
}

/// [`solid_model_scale`] floored at 1.0 — the characteristic length
/// [`KernelTolerances::for_solid`] and the boolean/imprint/heal sites are tuned
/// against.  The floor keeps a unit-and-below part on the historical bands those
/// call sites were fitted to; new size-relative sites should prefer the unfloored
/// [`model_scale`] family and floor explicitly where they mean to.
pub fn solid_scale(solid: &BrepSolid) -> f64 {
    solid_model_scale(solid).max(1.0)
}

// ---------------------------------------------------------------------------
// MEASURED tolerance — the deviation actually OBSERVED at construction
// ---------------------------------------------------------------------------

/// A deviation **measured** between a constructed entity and the geometry that
/// entity was built to reproduce, recorded next to the **derived** band it was
/// judged against.
///
/// # Why this type exists
///
/// Everything above this line in this module is *derived*: a band predicted
/// from a size (`model_scale`), a policy field, or a constant, computed
/// **before** the geometry exists.  Nothing above this line ever looks at what
/// the construction actually produced.  A collocation fit, a polyline
/// interpolation or a marched section can miss its intended locus by far more
/// (or, far more often, far less) than the derived band assumed, and today that
/// number is either thrown away or never taken.
///
/// This is the other half, and it is the half OCCT has and we did not:
/// `BRepOffset_SimpleOffset::FillEdgeData` sets an edge's tolerance to the
/// *measured* maximum distance between its new 3D curve and its pcurve on the
/// offset surface, over every adjacent face
/// (`BRepOffset_SimpleOffset.cxx:296-310`), and `UpdateTolerance`
/// (`BRepOffset_MakeOffset.cxx:4196-4302`) re-measures and raises after the
/// whole pipeline has run.  Their model throughout is *measure, do not
/// assume*: no tolerance in that pipeline is predicted from a size heuristic,
/// each is measured off the geometry that was actually built and then
/// propagated up the entity hierarchy.
///
/// # The direction rule — this may never loosen anything
///
/// A `MeasuredTolerance` is a **record of what happened, not a budget to
/// spend.**  The measured number may flow into
///
/// * a diagnostic,
/// * a refusal message,
/// * a comparison against a derived band that *tightens* an existing gate,
///
/// and nowhere else. It must never be `max`-ed into an acceptance band, a weld
/// radius or a search radius, because a construction would then certify its own
/// error: the sloppier the fit, the wider the band that judges it.
///
/// # Where it lives
///
/// Alongside the constructed entity, in the transient result struct the
/// construction already returns — never as a field on [`crate::BrepSolid`]'s
/// records. `BrepSolid` is serialized by `io/snapshot.rs`, which is a
/// documented durable format; a record field would be a format change with a
/// reader obligation. Persisting per-entity tolerances is a real and planned
/// piece of work with its own design (slice S3) — this type is the measurement
/// layer beneath it, and lands with no format churn at all.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MeasuredTolerance {
    deviation: f64,
    band: f64,
}

impl MeasuredTolerance {
    /// Record `deviation` against the derived `band` that was in force.
    ///
    /// Both arguments are normalised in the FAIL-SAFE direction, so a
    /// measurement that went wrong reads as "worse than the band", never as
    /// "fine": a non-finite or negative deviation becomes `+inf`, and a
    /// non-finite or negative band becomes `0.0`.  Either way
    /// [`Self::exceeds_band`] answers `true`.
    pub fn new(deviation: f64, band: f64) -> Self {
        let deviation = if deviation.is_finite() && deviation >= 0.0 {
            deviation
        } else {
            f64::INFINITY
        };
        let band = if band.is_finite() && band >= 0.0 {
            band
        } else {
            0.0
        };
        Self { deviation, band }
    }

    /// A construction with no fit to measure — a closed-form analytic result,
    /// or an affine offset that shifts a control net rigidly.  Records `0.0`
    /// deviation as a *claim*, which is why it is a named constructor: reading
    /// `MeasuredTolerance::exact(band)` at a call site says "this lane has no
    /// approximation error by construction", where `new(0.0, band)` would read
    /// as an unmeasured default.
    pub fn exact(band: f64) -> Self {
        Self::new(0.0, band)
    }

    /// What the construction actually did.
    pub fn deviation(&self) -> f64 {
        self.deviation
    }

    /// What the size-derived policy demanded of it.
    pub fn band(&self) -> f64 {
        self.band
    }

    /// The construction met the band it was judged against.
    pub fn within_band(&self) -> bool {
        self.deviation <= self.band
    }

    /// The construction was WORSE than the derived band assumed — the case
    /// worth acting on.  A site that gates on this refuses; a site that only
    /// reports names both numbers.
    pub fn exceeds_band(&self) -> bool {
        !self.within_band()
    }

    /// `deviation / band` — how much of the derived band the construction
    /// spent.  `> 1` is [`Self::exceeds_band`]; a value orders of magnitude
    /// below `1` says the band is loose here, which is worth knowing and is
    /// exactly what the corpus distribution reports.  A zero band answers `0.0`
    /// for a zero deviation and `+inf` otherwise.
    pub fn utilisation(&self) -> f64 {
        if self.band > 0.0 {
            self.deviation / self.band
        } else if self.deviation == 0.0 {
            0.0
        } else {
            f64::INFINITY
        }
    }

    /// The worse of two records: the larger deviation against the tighter band.
    ///
    /// Both halves move in the fail-safe direction, so folding a set of
    /// per-coedge or per-edge measurements can only ever make the summary
    /// harder to pass, never easier.
    pub fn worse_of(self, other: Self) -> Self {
        Self {
            deviation: self.deviation.max(other.deviation),
            band: self.band.min(other.band),
        }
    }

    /// Fold a set of measurements into their worst, or `None` when empty.
    pub fn worst(records: impl IntoIterator<Item = Self>) -> Option<Self> {
        records.into_iter().reduce(Self::worse_of)
    }

    /// `measured 1.234e-7 against band 4.000e-3` — the shared wording for
    /// refusal messages and diagnostics, so every site that reports a measured
    /// deviation reports both numbers in the same form.
    pub fn describe(&self) -> String {
        format!(
            "measured {:.3e} against band {:.3e}",
            self.deviation, self.band
        )
    }
}

/// Relative volume drift between two measurements of the same body that is
/// still the same answer: the case gate's band between a replay and its
/// baseline (`examples/case_replay.rs`), and the perturbation rescue's bar for
/// a rung against the exact lane's inclusion–exclusion identity
/// (`csg::boolean::perturbation_retry`). One number, so the rescue can never
/// accept a body the gate would call a different one. It only absorbs
/// last-bit summation noise.
pub const VOLUME_DRIFT_REL: f64 = 1e-9;

/// Absolute floor of the volume drift band, see [`VOLUME_DRIFT_REL`].
pub const VOLUME_DRIFT_ABS: f64 = 1e-9;

/// Fraction of the local characteristic length an APPROXIMATE offset
/// construction may deviate from the exact geometry it approximates — see
/// [`offset_construction_band`].
pub const OFFSET_CONSTRUCTION_REL: f64 = 5e-4;

/// Absolute floor of [`offset_construction_band`], so a vanishingly small part
/// is not held to a band below the arithmetic that builds it.
pub const OFFSET_CONSTRUCTION_FLOOR: f64 = 5e-6;

/// The band an approximate offset construction is judged against: 0.05% of the
/// caller's local characteristic length, floored at
/// [`OFFSET_CONSTRUCTION_FLOOR`].
///
/// This is a **fit-accuracy** band in this module's taxonomy — "how closely
/// must committed geometry agree?" — and it is the offset family's own
/// established answer, not a new number.  Three sites hand-roll exactly this
/// expression today: the ruled push's rim march
/// (`edit/direct_edit/face_offset.rs:303`, whose comment names it "the in-tree
/// precedent for how far an approximate offset result may be off"), the same
/// file's hole rebuild (`:1343`), and the free-form push's dense residual gate
/// (`edit/direct_edit/face_offset_freeform.rs:22`).  Naming it here gives the
/// MEASURED half of the tolerance model ([`MeasuredTolerance`]) the same bar the
/// gated half already uses, so a measurement and a gate on the same construction
/// cannot silently disagree.  The three copies are deliberately NOT converted
/// yet: two of those files are being rewritten by the general image-curve work
/// — the planned move to a general `image_curve(surface, pcurve)` transfer for
/// every pcurve, which retires the four iso-curve refusals at once — and a
/// drive-by edit there would collide for no behavioural gain; the expressions
/// are identical, so converting them later is a rename, not a change.
///
/// It is deliberately ~50x TIGHTER than
/// [`KernelTolerances::pcurve_acceptance`], which is where the same edge is
/// judged later by [`crate::BrepSolid::validate`].  The two answer different
/// questions: `pcurve_acceptance` asks "is this edge and this surface the same
/// entity?" and must absorb a vendor import's independent approximations, while
/// this asks "did OUR fit reproduce what we asked it to?" and has no vendor to
/// forgive.  A construction that passes here passes validate with two orders of
/// magnitude to spare; one that fails here is a bad fit even though validate
/// would still accept it, which is precisely the case a measured tolerance
/// exists to surface.
pub fn offset_construction_band(scale: f64) -> f64 {
    (scale.abs() * OFFSET_CONSTRUCTION_REL).max(OFFSET_CONSTRUCTION_FLOOR)
}

/// Propagate measured tolerances up one level of the entity hierarchy: from a
/// vertex's incident edges (and the gaps between their ends and the vertex
/// point) to the vertex itself.
///
/// The vertex's measured tolerance is the largest deviation any representation
/// meeting there exhibits: the worst endpoint gap `|p_V − c_E(t_end)|` over the
/// incident edge ends, and the worst measured deviation of those edges
/// themselves — because a point that sits exactly on a curve which is itself
/// `d` off its intended locus is `d` off that locus too.
///
/// # On OCCT's 1.001 factor — assessed, and deliberately NOT copied
///
/// `BRepOffset_SimpleOffset::FillVertexData` sets the vertex tolerance to
/// `1.001 × max(adjacent edge tolerances, endpoint spread)`
/// (`BRepOffset_SimpleOffset.cxx:398-424`).  That factor is not geometry.  OCCT
/// requires the ordering `Tol(V) ≥ Tol(E) ≥ Tol(F)` to hold as a *validity
/// invariant* on the persisted shape, re-checked by `BRepCheck_Vertex` /
/// `BRepCheck_Edge` and re-imposed by `BRepLib::UpdateTolerances` after
/// operations that recompute either side.  A vertex tolerance set exactly equal
/// to its edge's can be inverted by nothing more than a recomputation's last
/// bit, so they pad by a tenth of a percent.  It is an invariant fudge, and it
/// is theirs because their tolerances are persisted, grow-only, and re-derived
/// by checkers that compare them with `>`.
///
/// We have no such consumer.  Nothing in this kernel stores a per-entity
/// tolerance (see [`MeasuredTolerance`]'s "where it lives") and therefore
/// nothing re-derives one and compares it strictly against another.  Copying
/// the factor here would make the record *false* — it would report 0.1% more
/// deviation than was observed — for a benefit that does not exist.  So there is
/// no factor, and this function returns exactly the worst thing it saw.
///
/// If a later slice does persist these values and does enforce an ordering
/// invariant across a recompute, that slice should reintroduce a named padding
/// constant *at the invariant it protects*, with the comparison it protects
/// named — not here, and not silently.
pub fn vertex_tolerance_from_edges(
    endpoint_gaps: impl IntoIterator<Item = f64>,
    incident_edge_deviations: impl IntoIterator<Item = f64>,
) -> f64 {
    let sanitise = |value: f64| {
        if value.is_finite() && value >= 0.0 {
            value
        } else {
            f64::INFINITY
        }
    };
    endpoint_gaps
        .into_iter()
        .chain(incident_edge_deviations)
        .map(sanitise)
        .fold(0.0f64, f64::max)
}

