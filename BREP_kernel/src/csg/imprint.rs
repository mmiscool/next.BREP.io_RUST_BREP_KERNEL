use crate::classification::{parameter_point_in_face, PolygonClass};
use crate::curve::KNOT_IDENTITY_TOL;
use crate::spatial::{Aabb, Bvh};
use crate::tolerance::{
    assembler_weld, merge_scale, solid_scale, COINCIDENCE_DISTANCE_FLOOR, WELD_FLOOR,
};
use crate::topology::{BrepSolid, EdgeRecord, FaceRecord};
use crate::{
    build_pcurve_on_surface, classify_surface_pair_cached,
    fit_polyline, fit_polyline_of_degree, PolylineFit,
    intersect_curves, intersect_surfaces, intersect_surfaces_supplemental, project_point_to_curve,
    project_point_to_surface, project_point_to_surface_seeded, KnotVector, NurbsCurve, NurbsSurface,
    SurfaceClassifyData,
    SurfaceIntersectionOptions, SurfacePairClassification, SurfacePairRelation, Vec2, Vec3, Vec4,
};
use rustc_hash::{FxHashMap as HashMap, FxHashSet as HashSet};
use serde::{Deserialize, Serialize};
use std::time::Duration;
use web_time::Instant;

// Lane P3b row 3 (2026-10-26): the profile lines of one operation are
// gathered in a thread-local buffer and written to stderr ONCE when the
// outermost `ProfileSink` closes, instead of one unbuffered write per line
// (the `bench-occ` comparator timed our driver with `BREP_PROFILE=1` until
// row 5, and an outward box shell emitted ~435 lines). Line content is
// unchanged; lines emitted outside any open sink go straight to stderr.
thread_local! {
    static PROFILE_SINK: std::cell::RefCell<Option<String>> = const { std::cell::RefCell::new(None) };
}
#[doc(hidden)]
pub(crate) fn profile_emit(args: std::fmt::Arguments<'_>) {
    use std::fmt::Write;
    PROFILE_SINK.with(|sink| {
        let mut sink = sink.borrow_mut();
        match sink.as_mut() {
            Some(buffer) => {
                let _ = buffer.write_fmt(args);
                buffer.push('\n');
            }
            None => eprintln!("{args}"),
        }
    });
}
/// `eprintln!` for a `BREP_PROFILE` line: buffered while a [`ProfileSink`] is open.
macro_rules! profile_line {
    ($($arg:tt)*) => { $crate::imprint::profile_emit(format_args!($($arg)*)) };
}
/// Opens the operation's line buffer when `BREP_PROFILE` is set and no sink
/// is open on this thread (a nested operation writes into its parent's);
/// dropping the outermost sink writes the buffer in one call.
pub(crate) struct ProfileSink;
impl ProfileSink {
    pub(crate) fn open() -> Option<Self> {
        if !profile_enabled() {
            return None;
        }
        PROFILE_SINK.with(|sink| {
            let mut sink = sink.borrow_mut();
            if sink.is_some() {
                return None;
            }
            *sink = Some(String::with_capacity(16 * 1024));
            Some(Self)
        })
    }
}
impl Drop for ProfileSink {
    fn drop(&mut self) {
        if let Some(buffer) = PROFILE_SINK.with(|sink| sink.borrow_mut().take()) {
            if !buffer.is_empty() {
                eprint!("{buffer}");
            }
        }
    }
}

/// Per-step wall-clock accumulators for `build_imprints`, printed to stderr
/// when `BREP_PROFILE` is set. Native-only in practice (no env vars in wasm).
#[derive(Default)]
struct ImprintProfile {
    enabled: bool,
    pairs: u64,
    marched_pairs: u64,
    whole_reading_reuses: u64,
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
    hermite_runs: u64,
    fit_runs: u64,
    hermite: Duration,
    /// Appended at the end.
    pairs_fits: u64,
    exact_lane_fits: u64,
    /// Slice 1: planar-iso sections whose exact pcurves were handed to the
    /// builder, and those declined (a lifted chart, or no pcurve built).
    exact_iso_sections: u64,
    exact_iso_declined: u64,
}

impl ImprintProfile {
    fn new() -> Self {
        Self {
            enabled: profile_enabled(),
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
        profile_line!("imprint.whole_reading_reuses count={}", self.whole_reading_reuses);
        let ms = |d: Duration| d.as_secs_f64() * 1_000.0;
        profile_line!(
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
        profile_line!(
            "imprint.clip_fit rescue_march={:.2} clip={:.2} shared_boundary={:.2} fit={:.2} process={:.2} hermite={:.2} hermite_runs={} fit_runs={}",
            ms(self.rescue_march),
            ms(self.clip),
            ms(self.shared_boundary),
            ms(self.fit),
            ms(self.process),
            ms(self.hermite),
            self.hermite_runs,
            self.fit_runs,
        );
        profile_line!(
            "imprint.fits pairs={} exact_lanes={} exact_iso_sections={} exact_iso_declined={}",
            self.pairs_fits,
            self.exact_lane_fits,
            self.exact_iso_sections,
            self.exact_iso_declined,
        );
    }
}

/// Lane P3b row 0 (2026-10-26): where `process_curve`'s crossing loop, the
/// lies-on span search and the boundary-reuse pass spend their time, read only
/// under `BREP_PROFILE` and printed once per `build_imprints` as
/// `imprint.split`. Thread-local because the builder and the support
/// functions have no profile handle; `take` drains it for the report.
#[derive(Default, Clone, Copy)]
pub(crate) struct ImprintSplit {
    pub pierce: Duration,
    pub pierce_edges: u64,
    pub overlap: Duration,
    pub overlap_edges: u64,
    pub crossing: Duration,
    pub crossing_edges: u64,
    pub recover: Duration,
    pub recover_pierces: u64,
    pub lies_whole: Duration,
    pub lies_whole_calls: u64,
    pub lies_whole_hits: u64,
    pub lies_span: Duration,
    pub lies_span_calls: u64,
    pub reuse_edges: u64,
}
thread_local! {
    static IMPRINT_SPLIT: std::cell::Cell<ImprintSplit> = const { std::cell::Cell::new(ImprintSplit {
        pierce: Duration::ZERO, pierce_edges: 0, overlap: Duration::ZERO, overlap_edges: 0,
        crossing: Duration::ZERO, crossing_edges: 0, recover: Duration::ZERO, recover_pierces: 0,
        lies_whole: Duration::ZERO, lies_whole_calls: 0, lies_whole_hits: 0,
        lies_span: Duration::ZERO, lies_span_calls: 0, reuse_edges: 0,
    }) };
}
/// Whether `BREP_PROFILE` is set, read once per process (no test toggles it).
pub(crate) fn profile_enabled() -> bool {
    static ON: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *ON.get_or_init(|| std::env::var_os("BREP_PROFILE").is_some())
}
pub(crate) fn imprint_split_add(update: impl FnOnce(&mut ImprintSplit)) {
    IMPRINT_SPLIT.with(|cell| { let mut split = cell.get(); update(&mut split); cell.set(split); });
}
pub(crate) fn imprint_split_take() -> ImprintSplit {
    IMPRINT_SPLIT.with(|cell| cell.replace(ImprintSplit::default()))
}

/// Coarse observation of optional contact/graph calls, including early
/// declines. It owns no geometry and does not change caller diagnostics.
struct ImprintSubphaseClock {
    started: Option<Instant>,
    label: &'static str,
}
impl ImprintSubphaseClock {
    fn new(label: &'static str) -> Self {
        Self { started: profile_enabled().then(Instant::now), label }
    }
}
impl Drop for ImprintSubphaseClock {
    fn drop(&mut self) {
        if let Some(started) = self.started {
            profile_line!("imprint.subphase label={} ms={:.3}", self.label, started.elapsed().as_secs_f64() * 1000.0);
        }
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
    /// `None`, the default, marches exactly as before. `BREP_SPAN_BUDGET`
    /// overrides either way: `fit` is the pair's `intersection_fit` at the
    /// operands' scale, a number is an absolute budget, `off` is none.
    /// `BREP_SPAN_CENSUS=<file>` appends one line per clipped section with its
    /// stations, deviation and refine route (`span-budget census:`), labelled
    /// by `BREP_CENSUS_LABEL`.
    #[serde(default)]
    pub span_budget: Option<f64>,
    /// Implies a span budget of the pair's `intersection_fit` when none is
    /// set. Off by default; `BREP_HERMITE_SECTIONS` overrides either way (`0`
    /// off, anything else on).
    #[serde(default)]
    pub hermite_sections: bool,
    /// The TANGENT-CURVE lane: where an exact lane proves a pair's transverse
    /// intersection empty, or the tangential-only gate would skip it, and the
    /// pair classification reads near-tangent, look for an EXTENDED contact
    /// (`crate::tangent_curve`), march it with one planar carrier, and record it
    /// as a typed tangent section ([`ImprintResultRecord::tangent_sections`]).
    /// The boolean then names the contact in any refusal it raises on the pair
    /// instead of leaking the arrangement's own message. On by default since the
    /// census of 2026-10-10 (every manifest leaf identical with the lane on and
    /// off, `hlr_tests` green both ways); the section is recorded, never
    /// imprinted, so nothing the boolean builds changes.
    /// `BREP_TANGENT_CURVE_LANE=0` is the escape hatch. Appended at the end of
    /// the struct (2026-10-10).
    #[serde(default = "default_tangent_curve_lane")]
    pub tangent_curve_lane: bool,
    /// EXACT PCURVES FOR THE PLANAR-ISO LANE (slice 1, 2026-10-11, the
    /// catalogue's P3 applied to the exact lane): a section the planar-iso
    /// lane answers is an iso-curve of the ruled carrier and an affine image
    /// on the plane, so both pcurves are written down on the curve's own
    /// knots (`support::planar_iso_section`) instead of fitted, and the
    /// junction pass moves their ends instead of re-marching them. Off by
    /// default (the fitted pcurves are the shipped bytes);
    /// `BREP_EXACT_ISO_PCURVES` overrides either way: `0`/`off`, `on`, or
    /// `check`, which fits as before and appends one census line per pcurve
    /// comparing the fitted and the exact one in 3-D
    /// (`BREP_EXACT_ISO_CENSUS=<file>`, labelled by `BREP_CENSUS_LABEL`). A
    /// face read in a lifted chart declines the exact pcurve (the iso line
    /// is in the carrier's frame). Appended at the end of the struct.
    #[serde(default)]
    pub exact_iso_pcurves: bool,
    /// MARCH STARTS: every surface/surface march the imprint runs traces
    /// its starts in order (explicit trim-boundary pierces first, the most
    /// transverse first, a run-through start consumed with a snapped
    /// station) and seeds its interior candidates from the two carriers'
    /// knot-span leaves instead of a fixed grid
    /// ([`SurfaceIntersectionOptions::ordered_starts`],
    /// [`SurfaceIntersectionOptions::tree_seeds`]). Off by default (the
    /// grid-then-seeds march is the shipped one, byte for byte);
    /// `BREP_MARCH_STARTS` overrides either way: `0`/`off`, `order` (A3's
    /// ordering alone), `snap` (the ordering with run-through starts
    /// consumed), `tree` (A4 alone), `order+tree`, anything else all three.
    /// Appended at the end of the struct.
    #[serde(default)]
    pub march_starts: bool,
    /// Candidates for one junction are captured within the wide residual
    /// band; two combine only when they are connected (share a support face)
    /// and lie within the strict radius built from their own evidence radii;
    /// the surviving position is the lowest-dimensional candidate's, an
    /// operand vertex over an operand edge's crossing over a section end on
    /// two carriers. Off by default until the census in the record promotes
    /// it; `BREP_JUNCTION_COMBINE` overrides either way (`0`/`off`, `check`
    /// for the census that logs every junction's candidates to
    /// `BREP_JUNCTION_CENSUS` without moving anything, anything else on).
    /// Appended at the end of the struct (2026-10-11).
    #[serde(default)]
    pub two_radius_junctions: bool,
    /// IMPRINT THE TANGENT SECTION (the pinched result): a planar-carrier
    /// contact the tangent-curve lane traced is also IMPRINTED —
    /// `process_curve` on the clipped run, so the contact becomes a section
    /// piece on both carriers, the operand edges its ends lie on are split
    /// there, and every earlier piece the ends lie on (the through-bore's cap
    /// circles) is split at them — and its piece ids are recorded in
    /// [`TangentSectionRecord::piece_ids`]. The boolean's assembler then
    /// pairs the four coedges of such a piece by MATERIAL SIDE (two edge
    /// records on one locus, each manifold) instead of by arrival, so the
    /// grazing through-bore builds as one shell whose tangent edge is carried
    /// twice. On by default since the census of 2026-10-11 (the battery
    /// moving only the pins that asserted the old refusal and the recentre
    /// pin whose unguarded far fillet now completes);
    /// `BREP_TANGENT_CURVE_IMPRINT=0` is the escape hatch. Requires
    /// `tangent_curve_lane`. Appended at the end of the struct (2026-10-11).
    #[serde(default = "default_tangent_curve_imprint")]
    pub tangent_curve_imprint: bool,
}

fn default_tolerance() -> f64 {
    1e-7
}

fn default_tangent_curve_lane() -> bool {
    std::env::var("BREP_TANGENT_CURVE_LANE").as_deref() != Ok("0")
}

fn default_tangent_curve_imprint() -> bool {
    std::env::var("BREP_TANGENT_CURVE_IMPRINT").as_deref() != Ok("0")
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
            span_budget: None,
            hermite_sections: false,
            tangent_curve_lane: default_tangent_curve_lane(),
            tangent_curve_imprint: default_tangent_curve_imprint(),
            exact_iso_pcurves: false,
            march_starts: false,
            two_radius_junctions: false,
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
    /// Every EXTENDED tangential contact the tangent-curve lane recognised
    /// ([`ImprintOptions::tangent_curve_lane`]): the carriers touch along a
    /// curve inside both trims. A planar-carrier contact is marched and the
    /// clipped run described; a curved/curved contact is named at its point
    /// and not traced. None of these is imprinted yet — the record exists so
    /// the boolean can attribute a refusal on the pair to the contact by name.
    /// Appended at the end of the struct (2026-10-10).
    #[serde(default)]
    pub tangent_sections: Vec<TangentSectionRecord>,
}

/// One extended tangential contact the imprint recognised (see
/// [`ImprintResultRecord::tangent_sections`]).
#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct TangentSectionRecord {
    pub first: FaceKey,
    pub second: FaceKey,
    /// Ends of the longest run of the traced contact inside both trims (the
    /// contact point itself when the contact was not traced).
    pub start: Vec3,
    pub end: Vec3,
    /// Chord length of that run.
    pub length: f64,
    /// The run is the whole traced curve and the curve closed on itself.
    pub closed: bool,
    /// Stations of the run.
    pub stations: usize,
    /// One carrier is a plane: the branch the lane traces. `false` names a
    /// curved/curved contact the lane recognises but does not trace.
    pub planar: bool,
    /// The faces the contact INVOLVES: the two carriers' faces, and every face
    /// of either operand that shares an edge the run's ends lie on (the
    /// through-bore's contact line ends on the wall's top and bottom edges,
    /// so the box's top and bottom faces are involved; a blind pocket's ends
    /// lie on the bore's cap rings, so the caps are). The boolean's
    /// attribution rewrites a refusal only when a face it names is in here.
    #[serde(default)]
    pub faces: Vec<FaceKey>,
    /// The section pieces the contact was imprinted as
    /// ([`ImprintOptions::tangent_curve_imprint`]): `process_curve`'s pieces
    /// on the clipped run, in mint order. Empty when the section was recorded
    /// only. The assembler pairs the coedges of these pieces by material side.
    /// Appended at the end of the struct (2026-10-11).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub piece_ids: Vec<u64>,
    /// GRAZING PIERCES WITHOUT A SECTION (2026-10-17, the revolve-into-bore
    /// report): a trim edge of one face pierces the other carrier INSIDE both
    /// trims, but at an angle below the seed gate, and the march found no
    /// branch for the pair. The pierce proves a section crosses the faces
    /// there; the empty march says the crossing is too near tangency to walk
    /// (a third-order contact: the branch is born tangent to a contact curve
    /// on a shared boundary, `boolean::attribute_to_tangent_node`). Each
    /// entry is the pierce point and its `|edge_tangent · normal|`. Such a
    /// record is attribution only — nothing is imprinted — and names the
    /// contact when the assembly tears downstream of it. Appended at the end
    /// of the struct.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub grazing: Vec<(Vec3, f64)>,
}

impl TangentSectionRecord {
    /// A record standing on its grazing pierces ALONE — the lane found no
    /// contact to name, so the driver set its ends to the pierces (its
    /// `start` IS the first pierce, bit for bit, and nothing was traced). The
    /// boolean's attribution reads such a record only as a fallback: a
    /// contact the lane found, or a tangent node the imprint classified, is
    /// the nearer cause and keeps its own wording.
    pub fn pierce_only(&self) -> bool {
        self.stations == 0
            && self
                .grazing
                .first()
                .is_some_and(|(point, _)| point.sub(self.start).length() == 0.0)
    }
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
#[path = "imprint/hermite.rs"]
mod hermite;

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
pub(crate) use driver::{build_carrier_imprints, build_carrier_imprints_within, take_rescue_firings};
pub(crate) use self_touch::self_touch_edge_splits;
pub(crate) use support::clear_window_memo;
pub(crate) use self_touch::clear_touch_memo;


#[path = "imprint/pcurve_cache.rs"]
mod pcurve_cache;
use pcurve_cache::cached_build_pcurve as build_pcurve_on_surface_marched;

#[path = "imprint/intersection_cache.rs"]
mod intersection_cache;
use intersection_cache::intersect as intersect_curve_surface;
