//! # Semantic boolean oracle — an INDEPENDENT wrongness detector
//!
//! Repair-not-refuse proved the real boolean failures are STRUCTURAL: a result
//! that passes [`BrepSolid::validate`] (well-formed topology) yet is
//! geometrically WRONG — a missing face, a mis-selected fragment, an imprint
//! seam the assembler never welded.  `validate()` cannot see that; a
//! point-membership cross-check can.
//!
//! Two detectors live here, both purely diagnostic (never a hard gate in the
//! boolean hot path — see the `BREP_DEBUG_BOOL` hook in `boolean.rs`):
//!
//! * **Part 1 — semantic cross-check** ([`boolean_semantic_disagreement`]).
//!   After `A op B -> R`, sample points deterministically across the operands'
//!   combined box, classify each vs A, B, R with [`SolidClassifier`], and
//!   compare the CSG-expected membership (`UNION = in_a||in_b`,
//!   `INTERSECT = in_a&&in_b`, `SUBTRACT = in_a&&!in_b`) against R's own
//!   verdict.  Points on or near any boundary are SKIPPED (ambiguous), so a
//!   valid boolean scores ~0 disagreements and a structurally-wrong one scores
//!   a whole region's worth.
//!
//! * **Part 2 — fuse-after** ([`boolean_residual_fusables`]).  Geometry in R
//!   that is coincident within a weld band yet NOT topologically shared
//!   (distinct vertices at one point, duplicate coincident edges) — an
//!   intersector/assembler bug, because the weld should have fused it.
//!
//! Reliability is the whole point: a noisy oracle is worse than none.  The
//! On-skip band is size-coupled and deliberately conservative (a strict
//! superset of the classifier's `On` verdict via
//! [`SolidClassifier::within_band`]), and the flag threshold
//! ([`DISAGREEMENT_THRESHOLD`]) is tuned so every known-good fixture scores
//! zero.

use crate::classification::{PointClass, SolidClassifier};
use crate::spatial::Aabb;
use crate::topology::BrepSolid;
use crate::{BooleanOperation, KernelTolerances, Vec3};
use crate::{KernelRefusal, KernelStage, OrRefuse};
use serde::Serialize;

/// A boolean is FLAGGED structurally wrong when its decidable-point
/// disagreement rate exceeds this.  Tuned from the measured spread (fixed-seed,
/// so these numbers are stable): every KNOWN-GOOD boolean scores ≤ 0.7% (a
/// handful of stray points on the fused seam of tangent/glue unions), benign
/// doubly-curved cases (sphere/torus tubes) top out near 1.7%, while a genuine
/// structural defect — a mis-selected fragment, a lost overlap region, an
/// opened shell — flips a whole region and scores ≥ 7%.  The 3% line sits
/// cleanly in the gap: above the near-boundary noise the On-skip + threshold are
/// meant to absorb, well below any real wrongness.  One named constant so the
/// audit, the tests, and the stress binary share a single definition of "wrong".
pub const DISAGREEMENT_THRESHOLD: f64 = 0.03;

/// Fraction of the combined-box diagonal used as the conservative On-skip band.
/// Wide enough to swallow the few-micron near-coincidence gaps noisy operands
/// carry (glue/tangent fixtures) yet a negligible slice of the sampling volume.
const SKIP_FRACTION: f64 = 1e-4;

/// Fraction of the combined-box diagonal a boundary-focused sample is pushed off
/// its seed face along the surface normal.  Comfortably larger than
/// `SKIP_FRACTION` so the jittered point clears the On-skip band and reads a
/// clean In/Out, yet small enough to sit right against the boolean boundary
/// where a missing face is most discriminating.
const JITTER_FRACTION: f64 = 4e-3;

/// Share of the sample budget spent on bulk-volume points; the remainder is
/// spent near A/B/R faces (the discriminating region for a missing face).
const BULK_NUMERATOR: usize = 45;
const BULK_DENOMINATOR: usize = 100;

/// Deterministic `splitmix64` PRNG.  Reproducible, allocation-free, good enough
/// spread for Monte-Carlo point membership — we deliberately avoid `rand` /
/// wall-clock / `Math.random` so the audit is bit-stable across runs.
struct Rng {
    state: u64,
}

impl Rng {
    fn new(seed: u64) -> Self {
        Self { state: seed }
    }

    fn next_u64(&mut self) -> u64 {
        self.state = self.state.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.state;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    /// Uniform in `[0, 1)`.
    fn unit(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }

    /// Uniform in `[low, high)`.
    fn range(&mut self, low: f64, high: f64) -> f64 {
        low + (high - low) * self.unit()
    }

    /// A uniformly distributed unit vector on the sphere.
    fn unit_vector(&mut self) -> Vec3 {
        let z = self.range(-1.0, 1.0);
        let angle = self.range(0.0, std::f64::consts::TAU);
        let radius = (1.0 - z * z).max(0.0).sqrt();
        Vec3::new(radius * angle.cos(), radius * angle.sin(), z)
    }
}

/// One point where the result's membership contradicts the CSG expectation.
#[derive(Clone, Copy, Debug, Serialize)]
pub struct SemanticDisagreement {
    pub point: Vec3,
    /// CSG-expected membership of `point` in the result.
    pub expected_in: bool,
    /// The result solid's own In/Out verdict at `point`.
    pub result_in: bool,
    pub in_a: bool,
    pub in_b: bool,
}

/// Outcome of [`boolean_semantic_disagreement`].
#[derive(Clone, Debug, Serialize)]
pub struct OracleReport {
    /// Total points drawn.
    pub sampled: usize,
    /// Points skipped as On/near a boundary of A, B or R (ambiguous), or
    /// because a classifier could not decide them.
    pub on_skipped: usize,
    /// Decidable points actually compared (`sampled - on_skipped`).
    pub considered: usize,
    /// Every decidable point whose result membership contradicts CSG.
    pub disagreements: Vec<SemanticDisagreement>,
    /// `disagreements.len() / considered` (0 when nothing was decidable).
    pub disagreement_rate: f64,
}

impl OracleReport {
    /// Whether the disagreement rate crosses [`DISAGREEMENT_THRESHOLD`].
    pub fn is_flagged(&self) -> bool {
        self.disagreement_rate > DISAGREEMENT_THRESHOLD
    }

    /// A representative disagreeing point, if any.
    pub fn sample_disagreement(&self) -> Option<SemanticDisagreement> {
        self.disagreements.first().copied()
    }
}

fn faces_of(solid: &BrepSolid) -> impl Iterator<Item = &crate::topology::FaceRecord> {
    solid.shells.iter().flat_map(|shell| shell.faces.iter())
}

fn combined_bounds(first: &BrepSolid, second: &BrepSolid) -> Result<Aabb, KernelRefusal> {
    let mut bounds = Aabb::empty();
    for face in faces_of(first).chain(faces_of(second)) {
        bounds.include(
            Aabb::from_surface_controls(&face.surface)
                .or_refuse(KernelStage::Validate, "from_surface_controls")?,
        );
    }
    Ok(bounds)
}

/// Expected CSG membership of a point given its In-ness in each operand.
fn expected_membership(operation: BooleanOperation, in_a: bool, in_b: bool) -> bool {
    match operation {
        BooleanOperation::Union => in_a || in_b,
        BooleanOperation::Intersect => in_a && in_b,
        BooleanOperation::Subtract => in_a && !in_b,
    }
}

/// Part 1 — the semantic cross-check.  Sample points across the combined box of
/// `first ∪ second`, classify each versus the two operands and the result, and
/// flag every decidable point where the result membership disagrees with the
/// CSG expectation for `operation`.  See the module docs for the sampling /
/// On-skip / threshold design.
pub fn boolean_semantic_disagreement(
    first: &BrepSolid,
    second: &BrepSolid,
    operation: BooleanOperation,
    result: &BrepSolid,
    samples: usize,
) -> Result<OracleReport, KernelRefusal> {
    let policy = KernelTolerances::for_pair(first, second, 1e-7);
    let model = policy.model;

    let bounds = combined_bounds(first, second)?;
    let diagonal = bounds.diagonal();
    if !diagonal.is_finite() || diagonal <= 0.0 || samples == 0 {
        return Ok(OracleReport {
            sampled: 0,
            on_skipped: 0,
            considered: 0,
            disagreements: Vec::new(),
            disagreement_rate: 0.0,
        });
    }

    let classifier_a =
        SolidClassifier::new(first, model).or_refuse(KernelStage::Validate, "new")?;
    let classifier_b =
        SolidClassifier::new(second, model).or_refuse(KernelStage::Validate, "new")?;
    let classifier_r =
        SolidClassifier::new(result, model).or_refuse(KernelStage::Validate, "new")?;

    let skip_band = (model * 10.0).max(diagonal * SKIP_FRACTION);
    let jitter = diagonal * JITTER_FRACTION;

    // Flat face list for boundary-focused sampling — union of A, B and R faces
    // so every boolean boundary (operand carriers AND the result's own seams)
    // gets probed from both sides.
    let boundary_faces: Vec<&crate::topology::FaceRecord> = faces_of(first)
        .chain(faces_of(second))
        .chain(faces_of(result))
        .collect();

    let bulk_count = samples * BULK_NUMERATOR / BULK_DENOMINATOR;

    let mut rng = Rng::new(0x0B00_00AC_1E00_5EED);
    let mut report = OracleReport {
        sampled: 0,
        on_skipped: 0,
        considered: 0,
        disagreements: Vec::new(),
        disagreement_rate: 0.0,
    };

    for index in 0..samples {
        let point = if index < bulk_count || boundary_faces.is_empty() {
            Vec3::new(
                rng.range(bounds.minimum.x, bounds.maximum.x),
                rng.range(bounds.minimum.y, bounds.maximum.y),
                rng.range(bounds.minimum.z, bounds.maximum.z),
            )
        } else {
            boundary_sample(&mut rng, &boundary_faces, jitter)
        };
        report.sampled += 1;

        // On-skip: drop any point on or near a boundary of A, B or R. The
        // carrier-proximity probe is a strict superset of the classifier's On
        // verdict, so a kept point is guaranteed a clean In/Out below.
        let near = classifier_a
            .within_band(point, skip_band)
            .or_refuse(KernelStage::Validate, "within_band")?
            || classifier_b
                .within_band(point, skip_band)
                .or_refuse(KernelStage::Validate, "within_band")?
            || classifier_r
                .within_band(point, skip_band)
                .or_refuse(KernelStage::Validate, "within_band")?;
        if near {
            report.on_skipped += 1;
            continue;
        }

        let (Ok(ca), Ok(cb), Ok(cr)) = (
            classifier_a.classify(point),
            classifier_b.classify(point),
            classifier_r.classify(point),
        ) else {
            // A point the ray caster could not resolve cleanly — treat as
            // ambiguous rather than let it fabricate a disagreement.
            report.on_skipped += 1;
            continue;
        };
        // Every kept point cleared the On-skip band, so On should not occur;
        // if it somehow does, skip it (never fabricate a disagreement).
        if ca.class == PointClass::On || cb.class == PointClass::On || cr.class == PointClass::On {
            report.on_skipped += 1;
            continue;
        }

        let in_a = ca.class == PointClass::In;
        let in_b = cb.class == PointClass::In;
        let result_in = cr.class == PointClass::In;
        let expected_in = expected_membership(operation, in_a, in_b);
        report.considered += 1;
        if expected_in != result_in {
            report.disagreements.push(SemanticDisagreement {
                point,
                expected_in,
                result_in,
                in_a,
                in_b,
            });
        }
    }

    report.disagreement_rate = if report.considered == 0 {
        0.0
    } else {
        report.disagreements.len() as f64 / report.considered as f64
    };
    Ok(report)
}

/// Expected CSG membership of a point given its In-ness in each of the N
/// operands (`in_operands[k]` = point is In operand k).  Mirrors
/// [`expected_membership`] generalized to N solids:
/// `Union` = In ANY, `Intersect` = In ALL, `Subtract` = In operand 0 AND Out of
/// every other.
fn expected_membership_nary(operation: BooleanOperation, in_operands: &[bool]) -> bool {
    match operation {
        BooleanOperation::Union => in_operands.iter().any(|&inside| inside),
        BooleanOperation::Intersect => in_operands.iter().all(|&inside| inside),
        BooleanOperation::Subtract => {
            in_operands[0] && in_operands[1..].iter().all(|&inside| !inside)
        }
    }
}

/// N-ary semantic cross-check — the correctness gate for
/// [`crate::boolean_operation_nary`].  Sample points across the combined box of
/// all `operands`, classify each versus every operand and the result, and flag
/// every decidable point where the result membership disagrees with the n-ary
/// CSG expectation (`Union` = In any, `Intersect` = In all, `Subtract` = In
/// operand 0 and Out of the rest).  Points on or near ANY operand or result
/// boundary are skipped, so a correct n-ary boolean scores ~0.
pub fn boolean_semantic_disagreement_nary(
    operands: &[BrepSolid],
    operation: BooleanOperation,
    result: &BrepSolid,
    samples: usize,
) -> Result<OracleReport, KernelRefusal> {
    if operands.is_empty() {
        return Err(KernelRefusal::internal(
            KernelStage::Validate,
            "oracle",
            "boolean_semantic_disagreement_nary: no operands",
        ));
    }
    let model = KernelTolerances::for_solid(&operands[0], 1e-7).model;

    let mut bounds = Aabb::empty();
    for operand in operands {
        for face in faces_of(operand) {
            bounds.include(
                Aabb::from_surface_controls(&face.surface)
                    .or_refuse(KernelStage::Validate, "from_surface_controls")?,
            );
        }
    }
    let diagonal = bounds.diagonal();
    if !diagonal.is_finite() || diagonal <= 0.0 || samples == 0 {
        return Ok(OracleReport {
            sampled: 0,
            on_skipped: 0,
            considered: 0,
            disagreements: Vec::new(),
            disagreement_rate: 0.0,
        });
    }

    let classifiers = operands
        .iter()
        .map(|operand| SolidClassifier::new(operand, model))
        .collect::<Result<Vec<_>, _>>()
        .or_refuse(KernelStage::Validate, "csg.oracle")?;
    let classifier_r =
        SolidClassifier::new(result, model).or_refuse(KernelStage::Validate, "new")?;

    let skip_band = (model * 10.0).max(diagonal * SKIP_FRACTION);
    let jitter = diagonal * JITTER_FRACTION;

    let boundary_faces: Vec<&crate::topology::FaceRecord> = operands
        .iter()
        .flat_map(faces_of)
        .chain(faces_of(result))
        .collect();

    let bulk_count = samples * BULK_NUMERATOR / BULK_DENOMINATOR;

    let mut rng = Rng::new(0x0B00_00AC_1E00_5EED);
    let mut report = OracleReport {
        sampled: 0,
        on_skipped: 0,
        considered: 0,
        disagreements: Vec::new(),
        disagreement_rate: 0.0,
    };

    for index in 0..samples {
        let point = if index < bulk_count || boundary_faces.is_empty() {
            Vec3::new(
                rng.range(bounds.minimum.x, bounds.maximum.x),
                rng.range(bounds.minimum.y, bounds.maximum.y),
                rng.range(bounds.minimum.z, bounds.maximum.z),
            )
        } else {
            boundary_sample(&mut rng, &boundary_faces, jitter)
        };
        report.sampled += 1;

        // On-skip: drop any point near any operand's OR the result's boundary.
        let mut near = classifier_r
            .within_band(point, skip_band)
            .or_refuse(KernelStage::Validate, "within_band")?;
        for classifier in &classifiers {
            near = near
                || classifier
                    .within_band(point, skip_band)
                    .or_refuse(KernelStage::Validate, "within_band")?;
        }
        if near {
            report.on_skipped += 1;
            continue;
        }

        let Ok(cr) = classifier_r.classify(point) else {
            report.on_skipped += 1;
            continue;
        };
        if cr.class == PointClass::On {
            report.on_skipped += 1;
            continue;
        }
        let mut in_operands = Vec::with_capacity(classifiers.len());
        let mut ambiguous = false;
        for classifier in &classifiers {
            match classifier.classify(point) {
                Ok(classification) if classification.class != PointClass::On => {
                    in_operands.push(classification.class == PointClass::In);
                }
                _ => {
                    ambiguous = true;
                    break;
                }
            }
        }
        if ambiguous {
            report.on_skipped += 1;
            continue;
        }

        let result_in = cr.class == PointClass::In;
        let expected_in = expected_membership_nary(operation, &in_operands);
        report.considered += 1;
        if expected_in != result_in {
            report.disagreements.push(SemanticDisagreement {
                point,
                expected_in,
                result_in,
                // in_a/in_b carry the first two operands' membership for a
                // readable sample; the full vector is summarized by the counts.
                in_a: in_operands[0],
                in_b: *in_operands.get(1).unwrap_or(&false),
            });
        }
    }

    report.disagreement_rate = if report.considered == 0 {
        0.0
    } else {
        report.disagreements.len() as f64 / report.considered as f64
    };
    Ok(report)
}

/// Draw a boundary-focused sample: pick a face, a random point on its carrier,
/// and push it off the surface along the (randomly signed) normal by `jitter`.
fn boundary_sample(rng: &mut Rng, faces: &[&crate::topology::FaceRecord], jitter: f64) -> Vec3 {
    let face = faces[(rng.next_u64() as usize) % faces.len()];
    let (Ok([u0, u1]), Ok([v0, v1])) = (face.surface.domain_u(), face.surface.domain_v()) else {
        return Vec3::default();
    };
    let u = rng.range(u0, u1);
    let v = rng.range(v0, v1);
    let base = match face.surface.evaluate(u, v) {
        Ok(point) => point,
        Err(_) => return Vec3::default(),
    };
    let direction = match face.surface.normal(u, v) {
        Ok(normal) if normal.length() > 1e-9 => normal,
        _ => rng.unit_vector(),
    };
    let sign = if rng.unit() < 0.5 { -1.0 } else { 1.0 };
    base.add(direction.scale(sign * jitter))
}

/// A single residual fusable — coincident geometry the assembler left unwelded.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum ResidualKind {
    /// Two distinct vertex records at (within tolerance) the same point.
    CoincidentVertices,
    /// Two distinct edges tracing the same curve (coincident endpoints AND
    /// midpoint) that should be one shared edge.
    DuplicateEdge,
}

/// One [`boolean_residual_fusables`] finding.
#[derive(Clone, Debug, Serialize)]
pub struct ResidualFusable {
    pub kind: ResidualKind,
    pub detail: String,
    pub point: Vec3,
}

/// Part 2 — the fuse-after detector.  Any geometry in `result` that is
/// coincident within `tol` yet NOT topologically shared is an intersector /
/// assembler bug: the weld pass should have fused it.  Reports distinct
/// vertices at one point and duplicate coincident edges.  Purely diagnostic.
pub fn boolean_residual_fusables(result: &BrepSolid, tol: f64) -> Vec<ResidualFusable> {
    let mut findings = Vec::new();

    // Distinct vertices that coincide within `tol` — a weld that never fired.
    let vertices = &result.vertices;
    for i in 0..vertices.len() {
        for j in (i + 1)..vertices.len() {
            let gap = vertices[i].point.sub(vertices[j].point).length();
            if gap <= tol {
                findings.push(ResidualFusable {
                    kind: ResidualKind::CoincidentVertices,
                    detail: format!(
                        "vertices {} and {} coincide within {gap:.3e}",
                        vertices[i].id, vertices[j].id
                    ),
                    point: vertices[i].point,
                });
            }
        }
    }

    // Distinct edges tracing the same curve (both endpoints AND the midpoint
    // coincide) — a duplicated seam the assembler should have shared. The
    // midpoint test rejects a genuine two-edge bigon whose endpoints match but
    // whose interiors diverge.
    let edges = &result.edges;
    let endpoint =
        |edge: &crate::topology::EdgeRecord| -> Result<(Vec3, Vec3, Vec3), KernelRefusal> {
            let start = edge
                .curve
                .evaluate(edge.t0)
                .or_refuse(KernelStage::Validate, "evaluate")?;
            let end = edge
                .curve
                .evaluate(edge.t1)
                .or_refuse(KernelStage::Validate, "evaluate")?;
            let mid = edge
                .curve
                .evaluate((edge.t0 + edge.t1) * 0.5)
                .or_refuse(KernelStage::Validate, "evaluate")?;
            Ok((start, mid, end))
        };
    for i in 0..edges.len() {
        let Ok((si, mi, ei)) = endpoint(&edges[i]) else {
            continue;
        };
        for j in (i + 1)..edges.len() {
            let Ok((sj, mj, ej)) = endpoint(&edges[j]) else {
                continue;
            };
            let endpoints_match = (si.sub(sj).length() <= tol && ei.sub(ej).length() <= tol)
                || (si.sub(ej).length() <= tol && ei.sub(sj).length() <= tol);
            if endpoints_match && mi.sub(mj).length() <= tol {
                findings.push(ResidualFusable {
                    kind: ResidualKind::DuplicateEdge,
                    detail: format!(
                        "edges {} and {} trace the same curve within {tol:.3e}",
                        edges[i].id, edges[j].id
                    ),
                    point: mi,
                });
            }
        }
    }

    findings
}

