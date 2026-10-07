use crate::classification::{parameter_point_in_face, PolygonClass};
use crate::curve::KNOT_IDENTITY_TOL;
use crate::spatial::{Aabb, Bvh};
use crate::tolerance::{
    assembler_weld, merge_scale, solid_scale, COINCIDENCE_DISTANCE_FLOOR, WELD_FLOOR,
};
use crate::topology::{BrepSolid, EdgeRecord, FaceRecord};
use crate::{
    build_pcurve_on_surface, build_pcurve_on_surface_marched, classify_surface_pair_cached,
    fit_polyline, fit_polyline_of_degree, intersect_curve_surface, PolylineFit,
    intersect_curves, intersect_surfaces, intersect_surfaces_supplemental, project_point_to_curve,
    project_point_to_surface, project_point_to_surface_seeded, KnotVector, NurbsCurve, NurbsSurface,
    SurfaceClassifyData,
    SurfaceIntersectionOptions, SurfacePairClassification, SurfacePairRelation, Vec2, Vec3, Vec4,
};
use rustc_hash::{FxHashMap as HashMap, FxHashSet as HashSet};
use serde::{Deserialize, Serialize};
use std::time::Duration;
use web_time::Instant;

/// Per-step wall-clock accumulators for `build_imprints`, printed to stderr
/// when `BREP_PROFILE` is set. Native-only in practice (no env vars in wasm).
#[derive(Default)]
struct ImprintProfile {
    enabled: bool,
    pairs: u64,
    marched_pairs: u64,
    classify: Duration,
    cosurface: Duration,
    lies_on: Duration,
    planar_iso: Duration,
    analytic: Duration,
    seeds: Duration,
    march: Duration,
    clip_and_fit: Duration,
    process_curve: Duration,
    /// `clip_and_fit` split: the swapped-order rescue marches, the trim clip,
    /// the shared-boundary test, the polyline fit, and `process_curve`.
    rescue_march: Duration,
    clip: Duration,
    shared_boundary: Duration,
    fit: Duration,
    process: Duration,
}

impl ImprintProfile {
    fn new() -> Self {
        Self {
            enabled: std::env::var("BREP_PROFILE").is_ok(),
            ..Self::default()
        }
    }

    fn lap(&self, started: &mut Option<Instant>) -> Duration {
        if !self.enabled {
            return Duration::ZERO;
        }
        let now = Instant::now();
        let elapsed = started.map(|s| now - s).unwrap_or_default();
        *started = Some(now);
        elapsed
    }

    fn report(&self) {
        if !self.enabled {
            return;
        }
        let ms = |d: Duration| d.as_secs_f64() * 1_000.0;
        eprintln!(
            "imprint.profile pairs={} marched={} classify={:.2} cosurface={:.2} lies_on={:.2} planar_iso={:.2} analytic={:.2} seeds={:.2} march={:.2} clip_fit={:.2} process_curve={:.2}",
            self.pairs,
            self.marched_pairs,
            ms(self.classify),
            ms(self.cosurface),
            ms(self.lies_on),
            ms(self.planar_iso),
            ms(self.analytic),
            ms(self.seeds),
            ms(self.march),
            ms(self.clip_and_fit),
            ms(self.process_curve),
        );
        eprintln!(
            "imprint.clip_fit rescue_march={:.2} clip={:.2} shared_boundary={:.2} fit={:.2} process={:.2}",
            ms(self.rescue_march),
            ms(self.clip),
            ms(self.shared_boundary),
            ms(self.fit),
            ms(self.process),
        );
    }
}

#[derive(Clone, Debug, Deserialize)]
pub struct ImprintOptions {
    #[serde(default = "default_tolerance")]
    pub tolerance: f64,
    #[serde(default = "default_maximum_fit_points")]
    pub maximum_fit_points: usize,
    #[serde(default)]
    pub local_fit: bool,
    #[serde(default)]
    pub fit_chunk_points: Option<usize>,
    pub maximum_ssi_step: Option<f64>,
}

fn default_tolerance() -> f64 {
    1e-7
}

fn default_maximum_fit_points() -> usize {
    80
}

/// Angular band inside which two surface normals count as PARALLEL when a face
/// pair is classified (`|n_a × n_b| <= PAIR_ANGULAR_TOLERANCE` ⇒ the carriers
/// graze rather than cross there).  Single source for the pair classifier's
/// angular tolerance and for the near-tangency test that picks the march step.
const PAIR_ANGULAR_TOLERANCE: f64 = 1e-4;

/// Fraction of the part extent used as the FLOOR of the near-tangency march
/// step when the two carriers actually touch (`minimum_separation == 0`), so
/// the reachable curve length is a fixed fraction of the model rather than an
/// absolute distance.  See `march_maximum_step`.
const NEAR_TANGENT_STEP_FRACTION: f64 = 1e-3;

/// Span band for the B2 shared-section-edge decision, as a FRACTION of the part
/// extent (so it tracks feature size, not an absolute distance). A SSI section
/// is declared coincident with an existing boundary edge only when it lies
/// within `SHARED_SECTION_BAND_FRACTION * extent` of that edge along its whole
/// span in BOTH directions AND their endpoints already coincide within the weld
/// radius. Sized to admit a genuine near-tangent GRAZE (the marched/analytic
/// section drifts from the vendor edge by a graze-scale gap two-three orders
/// above the fit residual — the B1-falsification measurement) while staying far
/// below the feature scale at which a distinct same-endpoint edge diverges.
/// Measured on fixture 09 (extent ~3.23, band ~3.23e-3): the two genuine graze
/// gaps are 8.2e-4 and 1.67e-3 (ratios 2.5e-4 / 5.2e-4) — comfortably inside;
/// fixtures 12/04 (the wrong-reuse tripwires) produce ZERO reuses and the
/// prim×prim fuzz shows 0/2800 outcome changes, so the band is well separated
/// from any distinct-edge scale.
const SHARED_SECTION_BAND_FRACTION: f64 = 1e-3;

/// Maximum march step for one face pair.
///
/// A NEAR-TANGENT pair marches finely: where the carriers graze, the corrector
/// can hop between the two sheets of the contact, so the step stays inside the
/// separation band.  Two things that step must NOT be:
///
/// * **applied to a pair that only crosses.** A pair is classified `Singular`
///   when any sample near the contact has a DEGENERATE normal (a pole of a
///   revolution carrier) — that says nothing about tangency, and the pair can
///   cross at 90°.  Marching a transverse crossing at the tangency step crawls,
///   and a curve longer than `maximum_steps × step` then dies as "trace
///   exhausted" (2026-08-06 report: a draft-angle extrude wall against a
///   revolve-of-revolve face — a 2.0-unit curve needing 4014 steps of 4000,
///   10 points once marched at its own scale).  The step is therefore gated on
///   the classification's own tangency evidence, not on the relation label.
/// * **an absolute distance.** A fixed floor is a different fraction of every
///   model, so the reachable curve length depended on part size.  The floor is
///   a fraction of the part extent instead; `solid_scale` is at least 1, so
///   unit-scale parts keep the historical `1e-3` exactly.
fn march_maximum_step(
    classification: &SurfacePairClassification,
    requested: Option<f64>,
    scale: f64,
) -> Option<f64> {
    let grazes = matches!(
        classification.relation,
        SurfacePairRelation::NearTangent | SurfacePairRelation::Singular
    ) && classification.minimum_normal_cross <= PAIR_ANGULAR_TOLERANCE;
    if !grazes {
        return requested;
    }
    Some(
        requested.unwrap_or(
            classification
                .minimum_separation
                .max(NEAR_TANGENT_STEP_FRACTION * scale),
        ) * 0.5,
    )
}

impl Default for ImprintOptions {
    fn default() -> Self {
        Self {
            tolerance: default_tolerance(),
            maximum_fit_points: default_maximum_fit_points(),
            local_fit: false,
            fit_chunk_points: None,
            maximum_ssi_step: None,
        }
    }
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct ImprintVertex {
    pub id: u64,
    pub point: Vec3,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct FacePcurve {
    pub operand: u8,
    pub face_id: u64,
    pub pcurve: NurbsCurve,
}

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Eq, Hash, Serialize)]
pub struct FaceKey {
    pub operand: u8,
    pub face_id: u64,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct ImprintPieceRecord {
    pub id: u64,
    pub curve: NurbsCurve,
    pub t0: f64,
    pub t1: f64,
    pub start_vertex_id: u64,
    pub end_vertex_id: u64,
    pub pcurves: Vec<FacePcurve>,
    pub support_faces: [FaceKey; 2],
    /// OCCT common-block / `IsExistingPaveBlock` (B2): when this SSI section
    /// curve was found to COINCIDE along its whole span with an EXISTING
    /// boundary edge of one of its support faces, the section is not a new
    /// 1-cell — it IS that boundary edge. Records `(operand, edge_id, aligned)`
    /// of the existing edge to REUSE; `aligned` is whether the section's
    /// start→end runs the same direction as the edge's start→end. The
    /// assembler then resolves this piece to the shared boundary edge's
    /// identity so both operands reference ONE edge (no duplicate section /
    /// one-use edge). `None` = an ordinary freshly-minted section (the
    /// historical behaviour; also what `BREP_SHARED_SECTION_EDGE=0` forces).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub shared_edge: Option<(u8, u64, bool)>,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct EdgeSplitRecord {
    pub operand: u8,
    pub edge_id: u64,
    pub parameters: Vec<f64>,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct FaceImprints {
    pub operand: u8,
    pub face_id: u64,
    pub piece_ids: Vec<u64>,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct ImprintResultRecord {
    pub vertices: Vec<ImprintVertex>,
    pub pieces: Vec<ImprintPieceRecord>,
    pub by_face: Vec<FaceImprints>,
    pub edge_splits: Vec<EdgeSplitRecord>,
    /// Operand edges geometrically OVERLAPPED by a section curve (the
    /// cosurface/coincident locus: an inscribed sphere's contact circle
    /// riding a cap ring). Fragment-selection fate must never flood across
    /// them — the two sides of such an edge can lie on opposite sides of
    /// the other operand (the cap disc inside, the wall outside).
    #[serde(default)]
    pub barrier_edges: Vec<(u8, u64)>,
    /// Every ISOLATED TANGENT NODE the imprint admitted: a point where the two
    /// carriers touch with parallel normals and the section crosses itself, so
    /// the pair was marched rather than refused (see
    /// `imprint/tangent_contact.rs`). The boolean attributes a later tearing to
    /// these rather than reporting an anonymous degeneracy — the second-order
    /// filter cannot decide a third-order singularity, so the assembly is the
    /// authority on whether a node it admitted was really imprintable.
    #[serde(default)]
    pub tangent_nodes: Vec<Vec3>,
    /// True when the imprint saw ANY surface-contact evidence — an accepted
    /// pierce seed, a traced SSI branch (even one later clipped away), or a
    /// minted piece. Pure material disjointness leaves none, while a
    /// silently-lost section always leaves at least the upstream evidence, so
    /// the boolean's legitimate-empty adjudication requires this to be false.
    #[serde(default)]
    pub section_evidence: bool,
    /// Every face pair the imprint read as ONE surface and exchanged boundary
    /// curves across instead of intersecting (`cosurface_pair`, or the sampled
    /// pair classification's `Cosurface`). Selection holds a fragment whose
    /// point lies in such a partner's trim On against it, so the coincidence
    /// the sections were built on is the one the fragments are kept by.
    #[serde(default)]
    pub cosurface_pairs: Vec<(FaceKey, FaceKey)>,
}

#[path = "imprint/ruled_torus.rs"]
mod ruled_torus;
pub(crate) use ruled_torus::{circular_revolution_profile, construct_ruled_torus, RuledTorusBoundary};

#[path = "imprint/support.rs"]
mod support;
#[path = "imprint/builder.rs"]
mod builder;
#[path = "imprint/junctions.rs"]
mod junctions;
#[path = "imprint/self_touch.rs"]
mod self_touch;
#[path = "imprint/sections.rs"]
mod sections;
#[path = "imprint/tangent_contact.rs"]
mod tangent_contact;
#[path = "imprint/driver.rs"]
mod driver;

#[path = "imprint/gate_census.rs"]
mod gate_census;

use builder::*;
use junctions::*;
use sections::*;
use support::*;

#[path = "imprint/section_graph.rs"]
mod section_graph;
pub(crate) use section_graph::{plane_section_graph, plane_section_graph_range, plane_section_graph_range_bounded};

/// Reuse imprint's exact ruled-carrier section at the assembly repair boundary.
/// This exposes the construction, not its arrangement or acceptance policy.
pub(crate) fn planar_carrier_section(
    first: &NurbsSurface,
    second: &NurbsSurface,
    tolerance: f64,
) -> Result<Option<NurbsCurve>, crate::KernelRefusal> {
    support::planar_iso_intersection(first,second,tolerance)
}

use tangent_contact::classify_tangent_contact;

pub use driver::build_imprints;
pub(crate) use driver::build_carrier_imprints;
pub(crate) use self_touch::self_touch_edge_splits;
