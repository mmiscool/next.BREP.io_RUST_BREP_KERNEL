//! Conservative local BREP reconstruction from the kernel's public mesh segmentation.
//!
//! Plane and safely bounded cylinder/cone/sphere regions remain analytic.
//! Unsupported regions, and revolved regions whose interfaces cannot be shared
//! exactly, are represented by maximal edge-connected, provably coplanar planar
//! patches. A sphere is framed as a revolution about the common axis of the
//! rings that bound it; a cap around the pole carries the seam and the
//! degenerate pole edge of the kernel's own sphere solid. A sphere whose
//! plane rings lie on several axes, and two equal-radius cylinders whose axes
//! cross, share exact circles and ellipses whose pcurves are fitted; a
//! whole-body failure after such a lane demotes only its regions.

use brep_kernel::{
    make_sphere_surface_framed,
    build_pcurve_on_surface_range, circle_angle_to_parameter, intersect_analytic_pair, make_arc,
    make_line, make_revolution, project_point_to_curve, segment_mesh_faces,
    solid_signed_volume, ArenaCoedge, ArenaEdge, ArenaFace, ArenaLoop, ArenaShell, ArenaVertex,
    BrepSolid, EdgeId, FaceId, MeshRegion, MeshSegmentation, NurbsCurve, NurbsSurface, RegionCarrier,
    SegmentOptions,
    TopologyArena, Vec3, Vec4, KNOT_IDENTITY_TOL,
};
use crate::stl_conversion::{DemotedKind, DemotionCause, HybridDemotion};
use std::collections::{BTreeMap, BTreeSet, HashMap, VecDeque};
use std::f64::consts::TAU;

#[derive(Clone, Copy)]
struct PlaneInfo {
    origin: Vec3,
    normal: Vec3,
}

#[derive(Clone, Copy)]
struct CylinderCarrier {
    origin: Vec3,
    axis: Vec3,
    radius: f64,
    sense: i8,
    /// Largest radial deviation of a source vertex the segmenter accepted
    /// for this carrier: how far a ring vertex may sit off the exact circle.
    deviation: f64,
    /// [`HybridSwitches::pair_section_bar`], carried so every crossing test
    /// on this carrier agrees.
    pair_section_bar: bool,
}

#[derive(Clone, Copy)]
struct SphereCarrier {
    center: Vec3,
    radius: f64,
    sense: i8,
    /// Largest radial deviation of a source vertex the segmenter accepted
    /// for this carrier.
    deviation: f64,
}

/// A torus region's fitted carrier.
#[derive(Clone, Copy)]
struct TorusCarrier {
    center: Vec3,
    axis: Vec3,
    major_radius: f64,
    minor_radius: f64,
    sense: i8,
    /// Largest deviation of a source vertex the segmenter accepted for
    /// this carrier.
    deviation: f64,
}

#[derive(Clone, Copy)]
struct ConeCarrier {
    apex: Vec3,
    axis: Vec3,
    half_angle: f64,
    sense: i8,
    /// Largest deviation of a source vertex the segmenter accepted for
    /// this carrier.
    deviation: f64,
}

/// How far a ring or arc vertex may sit off its exact circle: the exact
/// topology tolerance, or twice the deviation the segmenter already
/// accepted for the carrier when that is wider (twice, because the carrier
/// is snapped to its neighbors after the acceptance was measured). A vertex
/// the tessellator left on a rim chord is still on the ring the exact
/// circle replaces.
fn accepted_radial_bar(tolerance: f64, deviation: f64) -> f64 {
    tolerance.max(2.0 * deviation)
}

struct CylinderInfo {
    carrier: CylinderCarrier,
    base_station: f64,
    height: f64,
    x_axis: Vec3,
    y_axis: Vec3,
    surface: NurbsSurface,
}

struct ConeInfo {
    carrier: ConeCarrier,
    /// The region reaches the apex: its surface starts there (station 0,
    /// v = 0) and its face closes through the seam and a degenerate apex
    /// edge, the way the kernel's own pointed cone solid is built.
    pointed: bool,
    base_station: f64,
    height: f64,
    x_axis: Vec3,
    y_axis: Vec3,
    surface: NurbsSurface,
}

struct SphereInfo {
    carrier: SphereCarrier,
    /// Polar axis of the revolution parameterization: the common normal of
    /// the ring planes bounding the region, pointing into the region; or,
    /// when the rings are on several axes, one whose poles both lie in
    /// removed caps (see [`tilted_sphere_frame`]).
    axis: Vec3,
    x_axis: Vec3,
    y_axis: Vec3,
    surface: NurbsSurface,
    /// The region is a cap around the north pole (one ring), so its face
    /// carries the seam and the degenerate pole edge.
    north_pole_inside: bool,
}

/// A torus band between coaxial rings, on the kernel's own torus template
/// (the tube circle starts at the outer equator and turns toward +axis), so
/// it recognizes as a torus and writes TOROIDAL_SURFACE. The frame axis is
/// the carrier's or its reverse, whichever keeps the band clear of the
/// template's tube seam; the seam meridian is a coaxial neighbour's, so a
/// ring shared with it is one parameter line on both faces.
struct TorusInfo {
    carrier: TorusCarrier,
    axis: Vec3,
    x_axis: Vec3,
    y_axis: Vec3,
    /// Tube angle at the band's middle, measured in the frame: the band's
    /// tube angles are unwrapped around it into [0, 2π].
    band_middle: f64,
    surface: NurbsSurface,
}

#[derive(Clone)]
struct Chain {
    vertices: Vec<usize>,
    closed: bool,
    adjacent: (u32, u32),
}

#[derive(Clone, Copy)]
struct Traversal {
    chain: usize,
    forward: bool,
}

struct RegionBoundary {
    cycles: Vec<Vec<Traversal>>,
}

struct LocalRegions {
    triangle_regions: Vec<u32>,
    planes: HashMap<u32, PlaneInfo>,
    cylinders: HashMap<u32, CylinderCarrier>,
    cones: HashMap<u32, ConeCarrier>,
    spheres: HashMap<u32, SphereCarrier>,
    tori: HashMap<u32, TorusCarrier>,
    local_to_original: HashMap<u32, u32>,
    count: usize,
    stats: HybridBrepStats,
}

enum HybridBuildFailure {
    /// No single parameter seam avoids all trims of this cylindrical wall.
    Periodic { original: u32 },
    Region { original: u32, message: String, cause: DemotionCause },
    /// The assembled shell failed a whole-body check after regions were
    /// built on a lane whose edges carry fitted pcurves on a sphere or
    /// cylinder (a tilted sphere ring, a crossing-bore ellipse). Those
    /// regions are demoted together and the build retried, so a new lane
    /// can only ever cost its own regions, never the body.
    Speculative { originals: Vec<u32>, message: String },
    Global(String),
}

impl From<String> for HybridBuildFailure {
    fn from(message: String) -> Self {
        Self::Global(message)
    }
}

impl From<&str> for HybridBuildFailure {
    fn from(message: &str) -> Self {
        Self::Global(message.to_owned())
    }
}

/// Auditable accounting for an analytic/faceted reconstruction.
#[derive(Clone, Debug, Default, Eq, PartialEq, serde::Serialize)]
pub(crate) struct HybridBrepStats {
    pub(crate) total_faces: usize,
    pub(crate) analytic_plane_faces: usize,
    pub(crate) analytic_plane_triangles: usize,
    pub(crate) analytic_cylinder_faces: usize,
    pub(crate) analytic_cylinder_triangles: usize,
    pub(crate) analytic_cone_faces: usize,
    pub(crate) analytic_cone_triangles: usize,
    pub(crate) analytic_sphere_faces: usize,
    pub(crate) analytic_sphere_triangles: usize,
    pub(crate) analytic_torus_faces: usize,
    pub(crate) analytic_torus_triangles: usize,
    pub(crate) faceted_faces: usize,
    pub(crate) faceted_triangles: usize,
    pub(crate) demoted_regions: usize,
    pub(crate) demoted_triangles: usize,
    pub(crate) segmentation_plane_regions: usize,
    pub(crate) segmentation_cylinder_regions: usize,
    pub(crate) segmentation_cone_regions: usize,
    pub(crate) segmentation_sphere_regions: usize,
    pub(crate) segmentation_torus_regions: usize,
    pub(crate) segmentation_unsupported_regions: usize,
    /// Lane J row 7: freeform regions the split divided into carriers,
    /// tried and left as they were, and above its budget (not tried).
    pub(crate) split_freeform_regions: usize,
    pub(crate) split_declined_regions: usize,
    pub(crate) split_over_budget_regions: usize,
    pub(crate) split_budget_hit_regions: usize,
    pub(crate) split_band_declined_regions: usize,
    /// [`HybridSwitches::gather_cone_strips`]: runs of cone strips gathered.
    pub(crate) gathered_cone_runs: usize,
    /// [`HybridSwitches::revolve_ends_first`]: line ends moved onto a vertex
    /// a revolved chain placed, and arcs given their ring's common station.
    pub(crate) moved_line_ends: usize,
    /// Lane J row 13: row 11's scan on the segmentation's whole sphere or
    /// torus carrier.
    pub(crate) whole_carrier_scan: crate::stl_conversion::WholeCarrierScanReport,
    /// Lane V row 1: why each demoted region was demoted.
    pub(crate) demotions: Vec<HybridDemotion>,
}

/// Validated reconstructed solid and the accounting used to select policy.
pub(crate) struct HybridBrepOutput {
    pub(crate) solid: BrepSolid,
    pub(crate) stats: HybridBrepStats,
}

#[derive(Clone, Copy)]
struct ChainUv {
    region: u32,
    start: [f64; 2],
    end: [f64; 2],
}

struct BuiltEdge {
    edge: EdgeId,
    curve_along_chain: bool,
    revolve_uvs: Vec<ChainUv>,
    custom_revolve_pcurves: HashMap<u32, NurbsCurve>,
}

struct Ids(u64);

impl Ids {
    fn next(&mut self) -> u64 {
        let value = self.0;
        self.0 += 1;
        value
    }
}

/// Rebuild a validated analytic/faceted shell from public segmentation.
///
/// This is intentionally narrower than a general mesh-to-BREP converter.
/// Plane and safely bounded cylinder/cone regions remain analytic. Unsupported
/// regions become maximal edge-connected coplanar planar patches. A cylinder touching
/// those facets is retained only when every shared edge is an exact axial
/// ruling; otherwise the complete cylinder region is recursively demoted to
/// planar triangle faces. Every shared boundary must still be provably a line,
/// circular arc, complete ring, or cylinder ruling.
pub(crate) fn hybrid_plane_cylinder_brep(
    positions: &[f64],
    indices: Option<&[u32]>,
    options: &SegmentOptions,
) -> Result<HybridBrepOutput, String> {
    hybrid_plane_cylinder_brep_retry(positions, indices, options, HybridSwitches::from_env(), None)
}

/// [`hybrid_plane_cylinder_brep`] without promotion 9's switches
/// (`gather_cone_strips`, `revolve_ends_first`): the STL conversion's second
/// try when the first rebuilt solid fails its STEP round trip.
pub(crate) fn hybrid_plane_cylinder_brep_before_promotion_9(
    positions: &[f64],
    indices: Option<&[u32]>,
    options: &SegmentOptions,
) -> Result<Option<HybridBrepOutput>, String> {
    let switches = HybridSwitches::from_env();
    if !switches.gather_cone_strips && !switches.revolve_ends_first {
        return Ok(None);
    }
    let without = HybridSwitches { gather_cone_strips: false, revolve_ends_first: false, ..switches };
    hybrid_plane_cylinder_brep_retry(positions, indices, options, without, None).map(Some)
}

/// [`hybrid_plane_cylinder_brep`] without promotion 10's switches
/// (`split_noise_bar`, `split_merge_refit`, `join_lone_freeform`,
/// `assemble_cone_bore`), and with `with_promotion_9` false without
/// promotion 9's too: the STL conversion's further tries when the rebuilt
/// solid fails its STEP round trip.
pub(crate) fn hybrid_plane_cylinder_brep_before_promotion_10(
    positions: &[f64],
    indices: Option<&[u32]>,
    options: &SegmentOptions,
    with_promotion_9: bool,
) -> Result<Option<HybridBrepOutput>, String> {
    let switches = HybridSwitches::from_env();
    if !(switches.split_noise_bar || switches.split_merge_refit || switches.join_lone_freeform || switches.assemble_cone_bore) {
        return Ok(None);
    }
    let mut without = HybridSwitches {
        split_noise_bar: false,
        split_merge_refit: false,
        join_lone_freeform: false,
        assemble_cone_bore: false,
        ..switches
    };
    if !with_promotion_9 {
        without.gather_cone_strips = false;
        without.revolve_ends_first = false;
    }
    hybrid_plane_cylinder_brep_retry(positions, indices, options, without, None).map(Some)
}

/// Localized failures demote one region per attempt; past this many, the
/// mesh is not a case of a few unsafe regions but of a segmentation the
/// builder cannot use, and the faceted fallback is the honest answer.
const HYBRID_RETRY_LIMIT: usize = 512;

/// The partition the mixed builder starts from: the kernel segmentation with
/// the fragmented cylinder/cone recovery applied, before any boundary is
/// validated or any region demoted. Exposed so a census can read the
/// carrier-accept stage the way the builder sees it.
pub fn segment_for_hybrid(
    positions: &[f64],
    indices: &[u32],
    options: &SegmentOptions,
) -> Result<MeshSegmentation, String> {
    // The census reads this per source triangle; the crease cut changes the
    // triangles.
    let switches = HybridSwitches { crease_cut: false, ..HybridSwitches::from_env() };
    segment_for_hybrid_measured(positions, indices, options, switches, &Forbidden::default())
        .map(|(segmentation, _, _, _)| segmentation)
}

/// Lane J row 7, promoted ON 2026-10-16: the freeform split and the crossing
/// fixes it relies on; BREP_HYBRID_SPLIT_FREEFORM=0 restores the old path.
fn split_freeform_enabled() -> bool {
    brep_kernel::switch!(live "BREP_HYBRID_SPLIT_FREEFORM", true)
}

/// Lane V's rebuild switches, read from the environment once per rebuild
/// (tests set them directly; `Default` is every switch off, the pre-lane-V
/// path). Rows 2a, 2b and 2c promoted ON 2026-10-29 as a set (census arm
/// `all3`: degraded 8.36 -> 7.20 % faceted, coarse 15.40 -> 6.48 %, no row
/// worse); each `=0` restores its old path. Promoted ON 2026-10-29 as a
/// second set (owner): row 2d (`absorb_revolve`), `absorb_retry`,
/// `pair_section_bar`, `whole_same_kind` and `whole_exact` (census arm
/// `b10all`: degraded 7.20 -> 4.97 % faceted, coarse 6.48 -> 0 %, 53 rows
/// fewer and none more, no false kind or parameter); each `=0` restores its
/// old path. Promoted ON 2026-10-29 as a third set (owner): `cap_ring`,
/// `crease_cut` and `shared_crossing` (census arm `b13shared` against the
/// promoted defaults: degraded 4.97 -> 4.29 % faceted, 7 rows fewer and none
/// more, no false kind or parameter); each `=0` restores its old path.
/// Promoted ON 2026-10-29 as a fourth set (owner): `sliver_lean`,
/// `whole_refit` and `crease_walls` (census arm `b18all` against the
/// promoted defaults: degraded 4.29 -> 3.62 % faceted, 12 rows fewer and
/// none more, no false kind or parameter, no volume flag); each `=0`
/// restores its old path. Promoted ON 2026-10-30 as a fifth set (owner):
/// `join_wall_pieces`, `absorb_chord`, `cone_refit`, `join_cone_pieces` and
/// `split_lone_cone` (census arm `b24all` against the promoted defaults:
/// degraded 3.62 -> 2.65 % faceted, 29 rows fewer and none more, no false
/// kind or parameter, no volume flag); each `=0` restores its old path.
/// Promoted ON 2026-10-30 as a sixth set (owner): `absorb_share`,
/// `join_cone_across` and `cone_seam` (census arm `b27all` against the
/// promoted defaults: degraded 2.65 -> 2.46 % faceted, 8 rows fewer and
/// none more, no false kind or parameter, no volume flag); each `=0`
/// restores its old path. Promoted ON 2026-10-30 as a seventh set (owner):
/// `split_cone_bore`, `split_probe_crease`, `absorb_curved_slivers` and
/// `noise_any_region` (census arm `b35all` against the promoted defaults:
/// degraded 2.46 -> 2.11 % faceted, 29 rows fewer and none more, no false
/// kind or parameter, no volume flag); each `=0` restores its old path.
/// Promoted ON 2026-10-31 as an eighth set (owner): `pointed_cone_apex`,
/// `absorb_repeat` and `frustum_two_rings` (census arm `b41all` against the
/// promoted defaults: degraded 2.11 -> 2.05 % faceted, 11 rows fewer and
/// none more, no false kind or parameter, no volume flag); each `=0`
/// restores its old path. Promoted ON 2026-10-31 as a ninth set (owner):
/// `merge_coplanar_planes`, `cone_stage_budget` and `plane_corners` (with
/// the STL conversion's `BREP_HYBRID_ROUNDTRIP_FALLBACK`; census pair
/// `p8on` against `p8off` in the promotion 8 hand-off), a volume error that
/// rises within its jittered siblings' range with no false kind or
/// parameter and the truth carrier unchanged no longer holding a switch
/// back; each `=0` restores its old path. Promoted ON 2026-10-31 as a tenth
/// set (owner): `gather_cone_strips` and `revolve_ends_first` (census pair
/// `p9on` against `p9off` in the promotion 9 hand-off); each `=0` restores
/// its old path. Promoted ON 2026-10-31 as an eleventh set (owner):
/// `cone_seam_bar` and `join_split_runs` (census pair `p10on` against
/// `p10off` in the promotion 10 hand-off); each `=0` restores its old path.
#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct HybridSwitches {
    /// Row 2a: the cap-fragment absorb runs on every noisy mesh, not only
    /// after the freeform split measured noise, at the builder's
    /// exact-topology bar (twice the measured noise). A jittered frustum kept
    /// one-triangle plane fragments on its cap rims; each split the rim into
    /// short chains no seam avoids, and the cone wall was demoted ("arc chain
    /// crosses the chosen seam"). BREP_HYBRID_ABSORB_UNSPLIT=1.
    pub(crate) absorb_unsplit: bool,
    /// Row 2b: a cylinder or cone ring or arc is the iso curve at the chain's
    /// mean station, so its bar holds each point to that station; the
    /// station spread is peak to peak, up to twice the deviation it stands
    /// for. BREP_HYBRID_RING_MEAN_BAR=1.
    pub(crate) ring_mean_bar: bool,
    /// Row 2c: a revolve region whose triangles fall in several
    /// edge-connected pieces becomes one region per piece, all on its
    /// carrier. The equal-radius crossed cylinders' walls each hold two arms
    /// that touch only at the crossing's two singular vertices; one face
    /// over both arms rebuilt as a genus-1 shell and the whole-body check
    /// demoted both walls. BREP_HYBRID_SPLIT_REVOLVE_PIECES=1.
    pub(crate) split_revolve_pieces: bool,
    /// Row 2d: a small region beside a cylinder, cone, sphere or torus at
    /// least eight times its size joins that region when the carrier holds
    /// every one of its vertices within the builder's bar and every
    /// triangle faces the carrier's way. One-triangle "planes" on a crossed
    /// wall, 20-70-triangle false cones and 4-triangle spheres on T-junction
    /// slivers each failed an edge with the wall, and the wall was demoted
    /// with them. BREP_HYBRID_ABSORB_REVOLVE=0
    /// restores the old path (promoted ON 2026-10-29).
    pub(crate) absorb_revolve: bool,
    /// Crossing cylinders (`cylinder_pair_section_bar`): the axis gap and an
    /// edge's offset from its section plane are held to the same bar as the
    /// radius, `accepted_radial_bar` of the exact-topology tolerance and the
    /// walls' deviation, instead of the bare tolerance. On chordal walls
    /// (T-junction vertices on chords, ~1e-3 deviation) the radius passed
    /// at 1.9e-3 while the section plane failed at 6.5e-5, and both walls
    /// were demoted (cause `cylinder_cylinder_edge`).
    /// BREP_HYBRID_PAIR_SECTION_BAR=0
    /// restores the old path (promoted ON 2026-10-29).
    pub(crate) pair_section_bar: bool,
    /// With row 2d: a host that took fragments and was demoted anyway is
    /// rebuilt with nothing joined to it (up to `ABSORB_RETRY_LIMIT` rounds,
    /// each forbidding every host demoted so far), and the rebuild with the
    /// fewest faceted triangles is kept. A joined wall can fail where its pieces
    /// passed (a jittered torus demoted whole, `torus_surface`; a pointed
    /// cone's `cone_span`), and a host demoted either way takes the
    /// fragments it absorbed with it: 48 census rows were worse with 2d.
    /// BREP_HYBRID_ABSORB_RETRY=0
    /// restores the old path (promoted ON 2026-10-29).
    pub(crate) absorb_retry: bool,
    /// A closed mesh the segmenter splits into several regions of one sphere
    /// or torus kind (plus freeform strays, and strays of other carriers each
    /// under 16 triangles and together at most 2 %) is offered to the whole-primitive fit as one
    /// region: `fit_whole_primitive` over every vertex but stitched
    /// T-junction vertices (`chord_vertices`), taken only when the residual
    /// reads as noise, then the genus and volume checks. A T-junction OCCT torus held its outer and inner
    /// halves as two tori (1 773 and 1 938 triangles) whose separate fits
    /// missed the shared latitude ring's coaxiality bar, and both halves
    /// were demoted (`torus_ring_edge`). BREP_HYBRID_WHOLE_SAME_KIND=0
    /// restores the old path (promoted ON 2026-10-29).
    pub(crate) whole_same_kind: bool,
    /// A closed mesh whose largest region is a sphere or torus is fitted
    /// whole beside strays of any carrier and size, and taken only when the
    /// fit is exact: every vertex but stitched T-junction ones
    /// (`chord_vertices`) within the weld tolerance. A real flat or cone
    /// leaves its vertices off the primitive; the kernel T-junction tori and
    /// spheres carried 30-86-triangle false cones on their chordal bands,
    /// and their exact fits read 1e-15. BREP_HYBRID_WHOLE_EXACT=0
    /// restores the old path (promoted ON 2026-10-29).
    pub(crate) whole_exact: bool,
    /// A cylinder edge on a plane square to its axis within 0.01 is judged
    /// as the builder builds it when the two regions' shared edges close
    /// into loops: `snap_cylinder_axes` squares the axis to the cap, so the
    /// ring is held to the radial bar about that axis. The edge check held
    /// the ring's station spread about the fitted axis to `tolerance * 4`;
    /// a noisy wall's axis tilts 4-8e-4 rad, moving a unit cap ring's
    /// station by ~1e-4 against a 6.4e-5 bar, and the crossed cylinders'
    /// walls fell against their own caps (`plane_neighbour_edge`).
    /// BREP_HYBRID_CAP_RING=0
    /// restores the old path (promoted ON 2026-10-29).
    pub(crate) cap_ring: bool,
    /// A freeform strip between two equal-radius crossing cylinder walls,
    /// every vertex within the pair's radial bar of one wall or the other,
    /// is cut along the walls' section ellipse (`cut_crossing_strips`): each
    /// edge from a vertex on one wall only to one on the other only is split
    /// where it crosses a section plane, at the ellipse point there, and
    /// every piece joins the wall its vertices lie on. The coarse facets
    /// straddling the crease shared no exact edge with either wall (a chord
    /// leaves the facet plane by its sag, 7e-5 to 2.3e-2 against a 1.6e-5
    /// bar) and both walls were demoted (`facet_neighbour_edge`).
    /// BREP_HYBRID_CREASE_CUT=0
    /// restores the old path (promoted ON 2026-10-29).
    pub(crate) crease_cut: bool,
    /// Cylinder regions that are pieces of one wall (axes parallel within
    /// 0.01, each origin within the radial bar of the other's axis line,
    /// radii within that bar) and that both cross the same third cylinder
    /// take one carrier: the mean axis, origin and radius
    /// (`share_crossing_wall_carriers`). The crossing's tangent point is then
    /// one point for both. A T-junction vertical wall held its arms above and
    /// below the horizontal wall as two fits 1.9e-5 apart; the vertex both
    /// crossings end at was pinned to one pair's tangent point, the other
    /// pair's two section ellipses then ended 2e-5 apart at it against a
    /// 1.6e-5 bar ("exact curves disagree"), and the retry faceted the row.
    /// BREP_HYBRID_SHARED_CROSSING=0
    /// restores the old path (promoted ON 2026-10-29).
    pub(crate) shared_crossing: bool,
    /// Row 2d's facing gate also allows a facet the lean its vertex noise
    /// can give it: asin of twice its worst vertex residual over its least
    /// altitude, at most the normal gate itself. A sliver 1e-2 high with
    /// vertices 1e-3 off the wall tilts by up to ~11 degrees on noise
    /// alone; a cap triangulated from rim vertices has large altitudes and
    /// residuals near zero, so it gains almost nothing. On census record
    /// kernel_crossed_cylinders_s10_d0.001_tjunction a 161-triangle false
    /// cylinder strip on the horizontal wall (worst vertex 2.6e-3 against a
    /// 1.9e-2 bar) was refused by three slivers 0.9-1.5e-2 high leaning
    /// 18.6-22.4 degrees against a 15 degree gate plus their spread.
    /// BREP_HYBRID_SLIVER_LEAN=0
    /// restores the old path (promoted ON 2026-10-29).
    pub(crate) sliver_lean: bool,
    /// The crease cut also takes a region of up to [`CREASE_SLIVER`]
    /// triangles fitted as anything but a cylinder, and counts cylinder
    /// regions that are pieces of one wall ([`same_cylinder_wall`]) as that
    /// wall: the region must border exactly two walls that cross. Each piece
    /// joins the member of its wall the source triangle borders (else the
    /// wall's largest). On census record
    /// occt_crossed_cylinders_s1_d0.005_tjunction each wall came as about
    /// five cylinder pieces of 20-64 triangles, with 4-triangle false tori
    /// and 2-triangle planes on the crease between them; every wall was
    /// demoted (`torus_ring_edge`, `plane_neighbour_edge`).
    /// BREP_HYBRID_CREASE_WALLS=0
    /// restores the old path (promoted ON 2026-10-29).
    pub(crate) crease_walls: bool,
    /// A whole sphere or torus the strays rule admits on the segmenter's
    /// carrier, whose deviation passes the weld tolerance, takes instead the
    /// whole fit without stitched chord vertices when that fit is exact
    /// (every vertex within the weld tolerance). The segmenter fits the
    /// chord vertices too: on census record occt_sphere_s1_d0.02_tjunction
    /// with [`HybridSwitches::sliver_lean`] a 7-triangle stray joined the
    /// sphere, the strays fell under 1 % of the mesh, and the carrier a
    /// 2.1e-2 deviation biased gave a volume 9.7e-5 off where the exact
    /// refit (radius 2.0, 4.4e-16) had been taken.
    /// BREP_HYBRID_WHOLE_REFIT=0
    /// restores the old path (promoted ON 2026-10-29).
    pub(crate) whole_refit: bool,
    /// Edge-adjacent cylinder regions that are pieces of one wall
    /// ([`same_cylinder_wall`]) become one region on the largest piece's
    /// carrier, before `split_revolve_pieces` (`join_wall_pieces`). On
    /// census record kernel_crossed_cylinders_s1_d0.001_jitter-0.1 the
    /// vertical wall came as pieces of 3 931 and 1 343 triangles (axes
    /// 4e-4 apart, origins within 4.7e-4 of each other's axis against a
    /// 1.1e-3 bar) meeting along 8 zig-zag mesh edges, and the horizontal
    /// wall likewise; each such edge is neither a ruling nor a crossing, and
    /// every wall was demoted.
    /// BREP_HYBRID_JOIN_WALL_PIECES=0
    /// restores the old path (promoted ON 2026-10-30).
    pub(crate) join_wall_pieces: bool,
    /// Row 2d's vertex bar skips stitched T-junction chord vertices
    /// (`chord_vertices`): such a vertex sits inside the wall by its chord's
    /// sag and reads nothing of it, and the host's deviation does not grow
    /// for it. On census record kernel_crossed_cylinders_s1_d0.02_tjunction
    /// 4-8 triangle freeform and plane slivers in the middle of the lower
    /// wall (deviation 2.75e-4) held chord vertices ~1e-2 inside it, row 2d
    /// refused them, and each demoted the wall (`plane_neighbour_edge`).
    /// A chord vertex is skipped only for a host that holds both ends of its
    /// chord (`chord_spans`): a vertex on a cone's ruling lies between its
    /// ruling neighbours too, and on census record
    /// occt_countersunk_hole_s1_d0.05_tjunction a cone sliver whose every
    /// off-ring vertex was such a vertex joined the bore (220 -> 308 faceted).
    /// BREP_HYBRID_ABSORB_CHORD=0
    /// restores the old path (promoted ON 2026-10-30).
    pub(crate) absorb_chord: bool,
    /// A cone region whose deviation passes the weld tolerance is refitted
    /// on its vertices other than stitched chord vertices, and takes that
    /// refit when it is exact (every such vertex within the weld tolerance;
    /// `refit_chord_cones`). The segmenter fits the chord vertices too: on
    /// census record occt_countersunk_hole_s1_d0.05_tjunction 7 chord
    /// vertices sit up to 1.7e-2 off the countersink, whose other 152 are
    /// within 2.2e-16, and the cone fitted at deviation 4.5e-3; its edge
    /// with the bore was refused (`cylinder_cone_edge`).
    /// BREP_HYBRID_CONE_REFIT=0
    /// restores the old path (promoted ON 2026-10-30).
    pub(crate) cone_refit: bool,
    /// Edge-adjacent cone regions whose union refits as one cone within the
    /// pair's radial bar become one region on that refit
    /// (`join_cone_pieces`), before `split_revolve_pieces`.
    /// BREP_HYBRID_JOIN_CONE_PIECES=0
    /// restores the old path (promoted ON 2026-10-30).
    pub(crate) join_cone_pieces: bool,
    /// The freeform split takes a region the cone stage finds to be one
    /// cone (the lone proposal is a cone whose support covers 19 in 20 of
    /// the region): strips the cone does not hold stay as they are, and row
    /// 7's cylinders that cover less than 19 in 20 of the region wait for
    /// the cone stage. On census record
    /// occt_countersunk_hole_s1_d0.001_tjunction the segmenter left the
    /// countersink with stray bore slivers as one 3 927-triangle freeform
    /// region; the cone stage fitted one cone holding 3 907 of them, and the
    /// split, which asked for two carriers, declined it.
    /// BREP_HYBRID_SPLIT_LONE_CONE=0
    /// restores the old path (promoted ON 2026-10-30).
    pub(crate) split_lone_cone: bool,
    /// Row 2d shares out a fragment no single host takes: each triangle goes
    /// to the eligible host (size, carrier, not forbidden) that holds it
    /// with the least residual, when every triangle has one. On census
    /// record kernel_countersunk_hole_s1_d0.001_jitter-0.1 a 5-triangle
    /// sphere sliver held full-height bore triangles and countersink
    /// triangles at the ring (worst 1.34 off the cone, 7.5e-2 off the bore);
    /// left alone, the cone was refused at its edge (`sphere_ring_edge`).
    /// BREP_HYBRID_ABSORB_SHARE=0
    /// restores the old path (promoted ON 2026-10-30).
    pub(crate) absorb_share: bool,
    /// [`HybridSwitches::join_cone_pieces`] also takes as candidates cone
    /// pieces that border one non-cone region of at most
    /// [`CONE_BRIDGE_TRIANGLES`] between them; row 2d then takes the
    /// slivers between the joined pieces (else `split_revolve_pieces` parts
    /// them again). On census record occt_countersunk_hole_s1_d0.005_jitter-0.1
    /// the countersink came as about 30 one-column strips of 15-16 triangles
    /// with sphere, freeform and plane slivers between them, no two strips
    /// edge-adjacent.
    /// BREP_HYBRID_JOIN_CONE_ACROSS=0
    /// restores the old path (promoted ON 2026-10-30).
    pub(crate) join_cone_across: bool,
    /// A cone face with no ruling or cylinder seam to follow takes as its
    /// seam the meridian of a boundary endpoint that avoids every open trim,
    /// as a cylinder's frame does; else its axis' arbitrary perpendicular.
    /// On census record occt_countersunk_hole_s1_d0.005_jitter-0.1 the
    /// joined countersink met the planar strips of a faceted bore along
    /// 2-point chains, and the arbitrary seam fell inside one ("arc chain ...
    /// crosses the chosen seam: u 0.988 -> 1.012").
    /// BREP_HYBRID_CONE_SEAM=0
    /// restores the old path (promoted ON 2026-10-30).
    pub(crate) cone_seam: bool,
    /// The freeform split's cone stage gives a cone it proposes alone that
    /// holds less than 19 in 20 of the region a cylinder on its axis through
    /// the rest, as a second carrier (`coaxial_bore`). On census record
    /// kernel_countersunk_hole_s1_d0.005_jitter-0.1 the countersink and 64
    /// bore triangles were one freeform region of 958; the cone stage fitted
    /// the countersink (894 triangles) and the bore only as 16-triangle
    /// pieces under the carrier floor, and the split declined the region.
    /// BREP_HYBRID_SPLIT_CONE_BORE=0
    /// restores the old path (promoted ON 2026-10-30).
    pub(crate) split_cone_bore: bool,
    /// A region the freeform split's stages divide nothing of is drawn
    /// again with each hypothesis from a probe that crosses no edge whose
    /// triangle normals differ by more than the normal tolerance
    /// (`RecognitionOptions::probe_crease_gate`) and first grows to 32
    /// triangles (`GATED_PROBE_TRIANGLES`). On census
    /// record occt_crossed_cylinders_s10_d0.001_jitter-0.3 both walls were one
    /// region of 788 long triangles, every 128-triangle probe spanned the
    /// crossing, and all 400 cylinder samples traced mixed the walls (radii
    /// 9-2 900 against 10, no seed triangle held); stage 1 accepted nothing
    /// and the region stayed faceted. On occt_crossed_cylinders_s1_d0.005_jitter-0.1
    /// each wall piece holds about 103 long triangles, and the default
    /// 128-triangle probe spanned two of them through the gate.
    /// BREP_HYBRID_SPLIT_PROBE_CREASE=0
    /// restores the old path (promoted ON 2026-10-30).
    pub(crate) split_probe_crease: bool,
    /// A torus, cone, cylinder or sphere fragment under 8 triangles that
    /// borders a plane region holding every one of its vertices within the
    /// bar plane and freeform fragments already join at
    /// (`absorb_plane_fragments`) joins that plane. On census record
    /// occt_frustum_s1_d0.005_jitter-1.0 a flat 4-triangle torus on the top
    /// cap (all at z 5.00) sat between the cap plane and the 1 638-triangle
    /// cone wall, and the wall was demoted at it (`torus_ring_edge`).
    /// BREP_HYBRID_ABSORB_CURVED_SLIVERS=0
    /// restores the old path (promoted ON 2026-10-30).
    pub(crate) absorb_curved_slivers: bool,
    /// When no plane region of at least 16 triangles measures the mesh's
    /// noise, the noisy-cap join takes it from the largest deviation of any
    /// analytic region that size (`measured_region_noise`). On census record
    /// occt_pointed_cone_s10_d0.005_jitter-1.0 the 61-triangle base came as
    /// 13 fragments of 1-12 triangles, the plane noise read 0, the join never
    /// ran, and the 1 691-triangle cone (deviation 4.8e-2) was demoted at the
    /// fragments (`plane_neighbour_edge`).
    /// BREP_HYBRID_NOISE_ANY_REGION=0
    /// restores the old path (promoted ON 2026-10-30).
    pub(crate) noise_any_region: bool,
    /// A cone snapped to the closed ring it shares with a plane takes the
    /// ring's centre from a least-squares circle fit in that plane
    /// (`ring_circle_center`) instead of the mean of the ring's points; and
    /// a cone none of whose vertices lies within its bar of the apex takes
    /// its apex at the vertex nearest its axis, with the opening refitted,
    /// when every vertex then holds within that bar (`apex_at_vertex`). On
    /// census record kernel_pointed_cone_s1_d0.001_jitter-0.1 the cone held
    /// two base triangles, its ring missed two of its 128 rim vertices, the
    /// mean sat 0.032 off the axis, and the apex lay 0.032 from the apex
    /// vertex (bar 7.3e-4); on jitter-0.3 the fitted opening (0.3746 against
    /// 0.3805) put the apex 0.087 beyond that vertex. Neither cone was taken
    /// as pointed (`cone_span`). The plane-ring snap also takes a fitted
    /// axis up to `RING_SNAP_TILT` off the plane normal when the ring is a
    /// circle about the normal within the cone's bar: on
    /// kernel_pointed_cone_s10_d0.005_jitter-0.3 the fitted axis lay 0.012
    /// off, the snap (0.01) was skipped, and the base ring read an axial
    /// spread of 0.50 (`chain_edge_cone`). BREP_HYBRID_POINTED_CONE_APEX=0
    /// restores the old path (promoted ON 2026-10-31).
    pub(crate) pointed_cone_apex: bool,
    /// The cap-fragment absorb (`absorb_plane_fragments`, after the noisy
    /// cap join) repeats while a pass joins a fragment, so a fragment that
    /// borders the cap plane only through fragments the last pass joined
    /// joins too. On census record occt_frustum_s10_d0.005_jitter-1.0 the
    /// top cap came as 13 fragments; one pass joined eight to the
    /// 8-triangle piece, and a 2-triangle fragment bordering it only through
    /// a 7-triangle one (under the 8-triangle host size), with three more
    /// beyond it, stayed apart; the cone wall failed at them
    /// (`arc chain crosses the chosen seam`). BREP_HYBRID_ABSORB_REPEAT=0
    /// restores the old path (promoted ON 2026-10-31).
    pub(crate) absorb_repeat: bool,
    /// A cone snapped to two closed rings it shares with parallel planes,
    /// whose first ring's frame misses the second ring by more than the
    /// cone's radial bar, takes its axis through the two ring centres (each
    /// from a circle fit, `ring_circle_center`) and its opening from both
    /// radii, when the line lies within 0.01 of the normal and the openings
    /// agree within 0.05 rad. On census record
    /// occt_frustum_s10_d0.005_jitter-1.0 read through the repaired mesh the
    /// top ring came first; its 43-triangle cap's normal tilted 1.2e-3 off,
    /// and 50 below it the base ring read a radial error of 0.106 against a
    /// bar of 0.094 (`chain_edge_cone`), while the record's ring lies within
    /// 0.04 of the true cone. Taken whenever two rings snapped (arm b40two),
    /// kernel_frustum_s10_d0.001_jitter-1.0 went from 3 faces to 3 389: its
    /// ring means sat off the axis and the centre line tilted the top ring
    /// past its station bar.
    /// BREP_HYBRID_FRUSTUM_TWO_RINGS=0
    /// restores the old path (promoted ON 2026-10-31).
    pub(crate) frustum_two_rings: bool,
    /// After the cap-fragment absorb, two edge-adjacent plane regions of at
    /// least 16 triangles each whose vertices all lie within twice the
    /// measured noise of their least-squares plane become one on it
    /// (`merge_coplanar_planes`). On census record
    /// occt_cylinder_s10_d0.005_jitter-1.0 the bottom cap came as two
    /// coplanar planes of 36 and 25 triangles, both past the fragment size
    /// the join and absorb take, and the cylinder was demoted where they
    /// meet (the cause behind ABSORB_CURVED_SLIVERS' whole-rebuild
    /// fallback). BREP_HYBRID_MERGE_COPLANAR_PLANES=0
    /// restores the old path (promoted ON 2026-10-31).
    pub(crate) merge_coplanar_planes: bool,
    /// Every chain end where only plane regions meet moves to the nearest
    /// point on all of their planes, within the exact-topology tolerance
    /// (`place_plane_corners`). The builder kept mesh vertex positions, so a
    /// jittered plane face's corners sat off its plane; the STEP import
    /// splits such a face into triangles through its vertices while the
    /// neighbouring edges stay where the builder put them, and on nine
    /// census countersinks the trims then missed their charts (`pcurve fit
    /// refused`) and the whole conversion failed (findings note
    /// 2026-10-31, option A). BREP_HYBRID_PLANE_CORNERS=0
    /// restores the old path (promoted ON 2026-10-31).
    pub(crate) plane_corners: bool,
    /// A cone the segmenter shredded into one region per strip (each fitted
    /// as whatever it read: sphere, torus, cone, plane or freeform) is
    /// gathered back into one cone region when one cone fitted to the run
    /// holds every vertex within the radial bar (`gather_cone_strips`). On
    /// census record occt_countersunk_hole_s10_d0.005_jitter-0.1 (jitter
    /// 0.05 of the split bar) the 738-triangle countersink came as about 48
    /// regions of 15-16 triangles, nearly all demoted, and the row kept 553
    /// faceted triangles. BREP_HYBRID_GATHER_CONE_STRIPS=0
    /// restores the old path (promoted ON 2026-10-31).
    pub(crate) gather_cone_strips: bool,
    /// The chain edges on cylinder, cone, sphere or torus regions are built
    /// before the lines between planes, and a line ends at the vertex such an
    /// edge already placed rather than at the mesh vertex. On census record
    /// occt_frustum_s1_d0.05_jitter-0.1 a line between two top-cap fragments
    /// placed a ring vertex at its mesh position; the cone's ring arc, the
    /// iso circle at the ring's mean station, then ended 5.2e-3 from it, the
    /// whole solid failed topology validation (`CurveVertexGap`), and the
    /// cone wall was demoted in the rebuild without CONE_SEAM; four occt and
    /// kernel frustum rows (440-478 faceted) fail the same way. Arcs of one
    /// cylinder or cone ring also share the ring's station
    /// (`common_arc_stations`): on occt_frustum_s10_d0.05_jitter-0.1 the top
    /// ring came as arcs between cap fragments, each the iso circle at its
    /// own mean station, and two of them ended 1.1e-2 apart at their shared
    /// vertex. BREP_HYBRID_REVOLVE_ENDS_FIRST=0
    /// restores the old path (promoted ON 2026-10-31).
    pub(crate) revolve_ends_first: bool,
    /// The freeform split's cone stage draws up to 2 048 attempts instead of
    /// 256 (`SPLIT_CONE_STAGE_BUDGET_ATTEMPTS`). On census record
    /// kernel_countersunk_hole_s10_d0.005_jitter-1.0 the stage ran out of its
    /// budget on the countersink region, proposed nothing, and the row kept
    /// 970 faceted triangles. BREP_HYBRID_CONE_STAGE_BUDGET=0
    /// restores the old path (promoted ON 2026-10-31).
    pub(crate) cone_stage_budget: bool,
    /// A cone's arc chain may end within the bar the cone's seam search
    /// allows (tolerance over the circumference at the wide end) of its
    /// seam, as a cylinder's, sphere's and torus's already may, instead of
    /// within 1e-8. The search put the seam through a jittered arc end and
    /// the builder then refused that end 1.6e-4 of a turn past it ("crosses
    /// the chosen seam") on census record
    /// kernel_countersunk_hole_s10_d0.02_jitter-0.1 (jitter 0.20 of the split
    /// bar), and the countersink was demoted with 180 faceted triangles.
    /// BREP_HYBRID_CONE_SEAM_BAR=0 restores the old path (promoted ON
    /// 2026-10-31).
    pub(crate) cone_seam_bar: bool,
    /// The noisy cap join (`join_noisy_planar_fragments`) cuts a run of small
    /// regions whose one plane fails at its creases (adjacent triangles more
    /// than 30 degrees apart) and joins each piece whose own plane holds. On
    /// census record occt_crossed_cylinders_s10_d0.02_jitter-0.3 three of the
    /// four 40-triangle caps came as freeform fragments linked through small
    /// wall pieces into one 127-triangle run, its plane missed by 10.3, none
    /// was joined, and the walls were demoted at them (444 faceted).
    /// BREP_HYBRID_JOIN_SPLIT_RUNS=0 restores the old path (promoted ON
    /// 2026-10-31).
    pub(crate) join_split_runs: bool,
    /// The freeform split's coaxial bore (`split_cone_bore`) also takes the
    /// small plane regions beside the region that it holds and the cone does
    /// not, and counts its pieces with them. On census record
    /// kernel_countersunk_hole_s1_d0.005_jitter-1.0 the segmenter cut 33 of
    /// the radius-1 bore's 96 triangles out as sixteen 2-triangle plane
    /// strips, the bore's other 63 sat in the countersink's region in pieces
    /// of 2-12 between them, the bore was refused under its 16-triangle
    /// piece floor, and the region stayed faceted (957 triangles).
    /// BREP_HYBRID_BORE_STRIPS=1 (default off).
    pub(crate) bore_strips: bool,
    /// A cone and the bore beneath it are assembled across the regions the
    /// segmenter and the split left them in. The freeform split's lone cone
    /// (`split_lone_cone`) takes a region it holds 3 in 4 of, not only 19 in
    /// 20, leaving the rest in the region; after the cone pieces are joined,
    /// `hybrid_freeform_split::assemble_cone_bore` fits one bore on its own
    /// axis through every square-facing triangle the cone's bordering
    /// fragments hold and hands each of them to the cone or the bore. On
    /// census record occt_countersunk_hole_s1_d0.005_jitter-1.0 the
    /// countersink and its bore were spread over freeform regions of 353,
    /// 159, 53 and 51 triangles and a dozen 15-triangle fragments, the
    /// cone held 92 and 94 % of the two largest (the split wants 19 in 20),
    /// and the cone's axis was tilted 3.9e-3 off the bore's (1.2e-2 over the
    /// bore against a bar of 6.8e-3), so a bore on it held 2 of 22.
    /// BREP_HYBRID_ASSEMBLE_CONE_BORE=0 restores the old path (promoted ON
    /// 2026-11-03 with the three below).
    pub(crate) assemble_cone_bore: bool,
    /// The whole-primitive path (`whole_closed_primitive`) admits, beside a
    /// dominant freeform region, strays each under 16 triangles (freeform
    /// ones at most 32) and together at most a tenth of the mesh, and leaves
    /// them to the whole fit's residual test. On census record occt_torus_s1_d0.1_jitter-1.0
    /// the segmenter chipped about 200 slivers of 1-10 triangles (2-triangle
    /// "spheres", 1-triangle planes, freeform pieces) off the torus's one
    /// freeform region of 3 285, 6.9 % of the mesh against row 10's 1 %,
    /// and the row was faceted (3 503 triangles).
    /// BREP_HYBRID_WHOLE_FREEFORM_STRAYS=1 (default off).
    pub(crate) whole_freeform_strays: bool,
    /// The first freeform split pass whose carriers' fits hold less than
    /// 19 in 20 of the region proposes once more with its cylinder fits of
    /// one wall refitted as one cylinder, and a region the passes decline
    /// takes that proposal
    /// (`hybrid_freeform_split::merge_refit_cylinders`); a merged wall's
    /// support is the region it holds at the merge bar, a region it carries
    /// is assigned at that bar, and an enclosed sliver group running along
    /// the carrier's axis joins it whatever its tilt. On census record
    /// kernel_crossed_cylinders_s1_d0.05_jitter-0.3 each crossing wall came
    /// as fits of 35-128 triangles, none a tenth of the region, and the row
    /// was faceted (2 772 triangles).
    /// BREP_HYBRID_SPLIT_MERGE_REFIT=0 restores the old path (promoted ON
    /// 2026-11-03).
    pub(crate) split_merge_refit: bool,
    /// The noisy cap join (`join_noisy_planar_fragments`) also takes a run
    /// that is one whole freeform region of at least 16 triangles whose
    /// least-squares plane holds every vertex within the measured noise: a
    /// cap the kernel segmenter kept whole instead of splitting into
    /// fragments. The join as it is takes only runs of two or more regions,
    /// so such a cap stayed freeform, the wall's rim edges were judged
    /// against its jittered facets' planes and failed the ring gate, and the
    /// builder demoted the wall. On census record
    /// kernel_crossed_cylinders_s10_d0.05_jitter-0.3 the bottom cap (30
    /// triangles, 0.13 off its plane against a 0.13 noise, 0.15 degrees off
    /// the wall's axis) was that region and the row stayed faceted (2 788
    /// triangles) with the merged walls.
    /// BREP_HYBRID_JOIN_LONE_FREEFORM=0 restores the old path (promoted ON
    /// 2026-11-03).
    pub(crate) join_lone_freeform: bool,
    /// With [`HybridSwitches::split_merge_refit`]: a region every freeform
    /// split pass leaves without a proposal, merged or not, is drawn once
    /// more for planes and cylinders at the merge bar (two split bars), and
    /// every cylinder fit of that pass is a wall at that bar (refitted with
    /// the fits of its wall, supported by what it holds at the bar) in a
    /// merged proposal the region takes. On census record
    /// kernel_crossed_cylinders_s1_d0.02_jitter-1.0 the vertices lie up to
    /// 0.016 off the walls against the 0.008 bar and the passes accepted no
    /// fit at all; the row was faceted (2 207 triangles).
    /// BREP_HYBRID_SPLIT_NOISE_BAR=0 restores the old path (promoted ON
    /// 2026-11-03).
    pub(crate) split_noise_bar: bool,
}

/// [`HybridSwitches::crease_walls`]: the largest non-cylinder region the
/// crease cut takes.
const CREASE_SLIVER: usize = 16;

/// [`HybridSwitches::pointed_cone_apex`]: how far (the sine of the angle) a
/// cone's fitted axis may lie off the normal of a plane it shares a closed
/// ring with and still snap to it, when that ring is a circle about the
/// normal within the cone's bar; 0.01 otherwise.
const RING_SNAP_TILT: f64 = 0.05;

/// [`HybridSwitches::join_cone_across`]: the largest region two cone pieces
/// may border between them and still be join candidates.
const CONE_BRIDGE_TRIANGLES: usize = 16;

impl HybridSwitches {
    pub(crate) fn from_env() -> Self {
        Self::from_lookup(|name| std::env::var(name).ok())
    }

    /// The switches the variables `lookup` returns select: a promoted row
    /// is on unless its variable is exactly "0", an opt-in row
    /// (`bore_strips`, `assemble_cone_bore`, `whole_freeform_strays`) only
    /// when it is exactly "1".
    fn from_lookup(lookup: impl Fn(&str) -> Option<String>) -> Self {
        let promoted = |name: &str| lookup(name).as_deref() != Some("0");
        let opted_in = |name: &str| lookup(name).as_deref() == Some("1");
        Self {
            absorb_unsplit: promoted("BREP_HYBRID_ABSORB_UNSPLIT"),
            ring_mean_bar: promoted("BREP_HYBRID_RING_MEAN_BAR"),
            split_revolve_pieces: promoted("BREP_HYBRID_SPLIT_REVOLVE_PIECES"),
            absorb_revolve: promoted("BREP_HYBRID_ABSORB_REVOLVE"),
            pair_section_bar: promoted("BREP_HYBRID_PAIR_SECTION_BAR"),
            absorb_retry: promoted("BREP_HYBRID_ABSORB_RETRY"),
            whole_same_kind: promoted("BREP_HYBRID_WHOLE_SAME_KIND"),
            whole_exact: promoted("BREP_HYBRID_WHOLE_EXACT"),
            cap_ring: promoted("BREP_HYBRID_CAP_RING"),
            crease_cut: promoted("BREP_HYBRID_CREASE_CUT"),
            shared_crossing: promoted("BREP_HYBRID_SHARED_CROSSING"),
            sliver_lean: promoted("BREP_HYBRID_SLIVER_LEAN"),
            crease_walls: promoted("BREP_HYBRID_CREASE_WALLS"),
            whole_refit: promoted("BREP_HYBRID_WHOLE_REFIT"),
            join_wall_pieces: promoted("BREP_HYBRID_JOIN_WALL_PIECES"),
            absorb_chord: promoted("BREP_HYBRID_ABSORB_CHORD"),
            cone_refit: promoted("BREP_HYBRID_CONE_REFIT"),
            join_cone_pieces: promoted("BREP_HYBRID_JOIN_CONE_PIECES"),
            split_lone_cone: promoted("BREP_HYBRID_SPLIT_LONE_CONE"),
            absorb_share: promoted("BREP_HYBRID_ABSORB_SHARE"),
            join_cone_across: promoted("BREP_HYBRID_JOIN_CONE_ACROSS"),
            cone_seam: promoted("BREP_HYBRID_CONE_SEAM"),
            split_cone_bore: promoted("BREP_HYBRID_SPLIT_CONE_BORE"),
            split_probe_crease: promoted("BREP_HYBRID_SPLIT_PROBE_CREASE"),
            absorb_curved_slivers: promoted("BREP_HYBRID_ABSORB_CURVED_SLIVERS"),
            noise_any_region: promoted("BREP_HYBRID_NOISE_ANY_REGION"),
            pointed_cone_apex: promoted("BREP_HYBRID_POINTED_CONE_APEX"),
            absorb_repeat: promoted("BREP_HYBRID_ABSORB_REPEAT"),
            frustum_two_rings: promoted("BREP_HYBRID_FRUSTUM_TWO_RINGS"),
            merge_coplanar_planes: promoted("BREP_HYBRID_MERGE_COPLANAR_PLANES"),
            cone_stage_budget: promoted("BREP_HYBRID_CONE_STAGE_BUDGET"),
            plane_corners: promoted("BREP_HYBRID_PLANE_CORNERS"),
            gather_cone_strips: promoted("BREP_HYBRID_GATHER_CONE_STRIPS"),
            revolve_ends_first: promoted("BREP_HYBRID_REVOLVE_ENDS_FIRST"),
            cone_seam_bar: promoted("BREP_HYBRID_CONE_SEAM_BAR"),
            join_split_runs: promoted("BREP_HYBRID_JOIN_SPLIT_RUNS"),
            bore_strips: opted_in("BREP_HYBRID_BORE_STRIPS"),
            assemble_cone_bore: promoted("BREP_HYBRID_ASSEMBLE_CONE_BORE"),
            whole_freeform_strays: opted_in("BREP_HYBRID_WHOLE_FREEFORM_STRAYS"),
            split_merge_refit: promoted("BREP_HYBRID_SPLIT_MERGE_REFIT"),
            join_lone_freeform: promoted("BREP_HYBRID_JOIN_LONE_FREEFORM"),
            split_noise_bar: promoted("BREP_HYBRID_SPLIT_NOISE_BAR"),
        }
    }
}

/// The hybrid segmentation, what the freeform split did (its measured noise
/// is zero when it wrote no carrier), under row 2d the triangles of each
/// host that took fragments (by its id before the join; `forbid` names hosts
/// that may take none and regions the widened cut may not take), and under [`HybridSwitches::crease_cut`] the cut
/// mesh (positions, indices, each triangle's source triangle) the
/// segmentation then indexes.
fn segment_for_hybrid_measured(
    positions: &[f64],
    indices: &[u32],
    options: &SegmentOptions,
    switches: HybridSwitches,
    forbid: &Forbidden,
) -> Result<(MeshSegmentation, crate::hybrid_freeform_split::SplitReport, BTreeMap<u32, Vec<usize>>, Option<CutMesh>), String> {
    let mut segmentation = segment_mesh_faces(positions, indices, options)?;
    crate::hybrid_recovery::recover(positions, indices, options, &mut segmentation);
    // Lane J row 7, promoted ON: a freeform region that holds several
    // carriers is split into them (crossing cylinder walls under vertex
    // noise). BREP_HYBRID_SPLIT_FREEFORM=0 restores the unsplit region.
    let mut split = if split_freeform_enabled() {
        crate::hybrid_freeform_split::split(
            positions,
            indices,
            options,
            &mut segmentation,
            switches.split_lone_cone,
            switches.split_cone_bore,
            switches.split_probe_crease,
            switches.cone_stage_budget,
            switches.bore_strips,
            switches.assemble_cone_bore,
            switches.split_merge_refit,
            switches.split_noise_bar,
        )
    } else {
        Default::default()
    };
    let split_noise = split.noise;
    // Lane J row 5, promoted ON: BREP_HYBRID_NOISE_PLANES=0 restores the
    // unjoined fragments.
    if brep_kernel::switch!(live "BREP_HYBRID_NOISE_PLANES", true) {
        let mut noise = measured_plane_noise(&segmentation).max(split_noise);
        if noise == 0.0 && switches.noise_any_region {
            noise = measured_region_noise(&segmentation);
        }
        join_noisy_planar_fragments(&mut segmentation, positions, indices, noise, split_noise > 0.0, switches.join_split_runs, switches.join_lone_freeform);
        if switches.absorb_unsplit {
            // The builder's exact-topology bar (twice the plane noise): a
            // rim fragment is split off because a vertex sits past the
            // cap's own largest residual.
            // Under absorb_repeat, a fragment that borders the cap only
            // through fragments a pass joined takes another pass.
            while crate::hybrid_freeform_split::absorb_plane_fragments(
                &mut segmentation,
                positions,
                indices,
                2.0 * noise,
                switches.absorb_curved_slivers,
            ) && switches.absorb_repeat
            {}
        } else if split_noise > 0.0 {
            crate::hybrid_freeform_split::absorb_plane_fragments(
                &mut segmentation,
                positions,
                indices,
                noise,
                switches.absorb_curved_slivers,
            );
        }
        // After the absorb, a cap is in its final pieces.
        if switches.merge_coplanar_planes {
            merge_coplanar_planes(&mut segmentation, positions, indices, 2.0 * noise);
        }
    }
    let mut absorbed = BTreeMap::new();
    let vertices = positions.chunks_exact(3).map(|p| Vec3::new(p[0], p[1], p[2])).collect::<Vec<_>>();
    let diagonal = mesh_diagonal(&vertices);
    // The exact-topology bar hybrid_plane_cylinder_brep_once uses.
    let mut tolerance = (options.fit_tolerance * diagonal).min(2.0e-6 * diagonal).max(1.0e-10);
    if brep_kernel::switch!(live "BREP_HYBRID_MESH_NOISE", true) {
        tolerance = tolerance.max(2.0 * measured_plane_noise(&segmentation).max(split_noise));
    }
    if switches.gather_cone_strips {
        split.gathered_cone_runs = gather_cone_strips(&mut segmentation, positions, indices, tolerance, switches.split_noise_bar.then_some(options.weld_tolerance));
    }
    if switches.join_cone_pieces {
        join_cone_pieces(&mut segmentation, positions, indices, tolerance, options.weld_tolerance, switches.join_cone_across);
    }
    if switches.assemble_cone_bore {
        crate::hybrid_freeform_split::assemble_cone_bore(&mut segmentation, positions, indices, options);
    }
    if switches.absorb_revolve {
        let cos_normal_gate = options.normal_tolerance_deg.to_radians().cos();
        let chord = if switches.absorb_chord {
            crate::hybrid_whole_freeform::chord_spans(positions, indices, options.weld_tolerance)
        } else {
            BTreeMap::new()
        };
        absorbed = absorb_revolve_fragments(
            &mut segmentation,
            &vertices,
            indices,
            tolerance,
            diagonal,
            cos_normal_gate,
            switches.sliver_lean,
            &chord,
            &forbid.hosts,
            switches.absorb_share,
        );
    }
    let cut = if switches.crease_cut {
        let chord = if switches.crease_walls {
            crate::hybrid_whole_freeform::chord_vertices(positions, indices, options.weld_tolerance)
        } else {
            BTreeSet::new()
        };
        cut_crossing_strips(&mut segmentation, positions, indices, tolerance, switches.crease_walls, &chord, &forbid.cuts)
    } else {
        None
    };
    if switches.join_wall_pieces {
        join_wall_pieces(&mut segmentation, cut.as_ref().map_or(indices, |cut| cut.indices.as_slice()), tolerance);
    }
    if switches.split_revolve_pieces {
        split_revolve_pieces(&mut segmentation, cut.as_ref().map_or(indices, |cut| cut.indices.as_slice()));
    }
    if switches.cone_refit {
        let (positions, indices) = cut.as_ref().map_or((positions, indices), |cut| (cut.positions.as_slice(), cut.indices.as_slice()));
        refit_chord_cones(&mut segmentation, positions, indices, options.weld_tolerance);
    }
    if std::env::var("BREP_DEBUG_SPLIT").is_ok() {
        for r in &segmentation.regions {
            eprintln!("[seg] {} {} tris {} dev {:.2e} bbox ({:.2},{:.2},{:.2})..({:.2},{:.2},{:.2})", r.id, r.carrier.kind(), r.triangle_count, r.max_deviation, r.bbox_min.x, r.bbox_min.y, r.bbox_min.z, r.bbox_max.x, r.bbox_max.y, r.bbox_max.z);
        }
    }
    Ok((segmentation, split, absorbed, cut))
}

/// The mesh [`HybridSwitches::crease_cut`] leaves: a cut triangle keeps its
/// index for its first piece and appends the others, each naming its source
/// triangle in `parent`.
pub(crate) struct CutMesh {
    positions: Vec<f64>,
    indices: Vec<u32>,
    parent: Vec<usize>,
    /// Under [`HybridSwitches::crease_walls`], the source triangles of each
    /// region cut only because of that switch.
    widened: Vec<Vec<usize>>,
}

/// What a rebuild under [`HybridSwitches::absorb_retry`] may not join: row
/// 2d hosts (by id before the join) and, under
/// [`HybridSwitches::crease_walls`], regions holding these source triangles
/// that the widened crease cut would take.
#[derive(Clone, Debug, Default)]
struct Forbidden {
    hosts: BTreeSet<u32>,
    cuts: BTreeSet<usize>,
}

/// [`HybridSwitches::crease_cut`]: cut every freeform region that borders
/// exactly two cylinder regions forming an equal-radius crossing
/// (`cylinder_pair_section`) and whose vertices each lie within the pair's
/// radial bar of one of them. An edge from a vertex on one wall only to one
/// on the other only is split where it crosses a section plane, at the point
/// of that plane's ellipse (the curve `cylinder_pair_section_edge` builds)
/// at the crossing's in-plane angle; a region is left whole when such an edge
/// borders a triangle outside it, crosses both planes or none, or its
/// ellipse point lies off the edge by more than the bar. Pieces join the
/// wall of their vertices on one wall only (a piece with none takes the wall
/// its vertices are nearer). `None` when nothing was cut. Under
/// [`HybridSwitches::crease_walls`] the candidates and walls widen as that
/// switch says, and a vertex in `chord` (empty otherwise) need lie on
/// neither wall; a region so widened is not cut when it holds a triangle in
/// `forbid`.
fn cut_crossing_strips(
    segmentation: &mut MeshSegmentation,
    positions: &[f64],
    indices: &[u32],
    tolerance: f64,
    crease_walls: bool,
    chord: &BTreeSet<u32>,
    forbid: &BTreeSet<usize>,
) -> Option<CutMesh> {
    let triangle_count = segmentation.triangle_region_ids.len();
    if indices.len() != triangle_count * 3 {
        return None;
    }
    let point = |v: u32| Vec3::new(positions[3 * v as usize], positions[3 * v as usize + 1], positions[3 * v as usize + 2]);
    let mut edges = HashMap::<(u32, u32), Vec<usize>>::new();
    for t in 0..triangle_count {
        for c in 0..3 {
            let (a, b) = (indices[3 * t + c], indices[3 * t + (c + 1) % 3]);
            edges.entry((a.min(b), a.max(b))).or_default().push(t);
        }
    }
    let mut members = BTreeMap::<u32, Vec<usize>>::new();
    for (t, &r) in segmentation.triangle_region_ids.iter().enumerate() {
        members.entry(r).or_default().push(t);
    }
    let cylinder_of = |r: u32| {
        segmentation.regions.get(r as usize).and_then(|region| carrier_cylinder(region, true).ok().flatten())
    };
    let mut new_positions = positions.to_vec();
    let mut new_indices = indices.to_vec();
    let mut parent = (0..triangle_count).collect::<Vec<_>>();
    let mut labels = segmentation.triangle_region_ids.clone();
    let mut cut_any = false;
    // Under crease_walls, each wall a cut joined: its pieces become one
    // region (by the lowest id).
    let mut joined_walls = BTreeMap::<u32, u32>::new();
    let mut widened = Vec::new();
    for (&region, triangles) in &members {
        let Some(carrier) = segmentation.regions.get(region as usize).map(|r| &r.carrier) else { continue };
        let sliver =
            crease_walls && triangles.len() <= CREASE_SLIVER && !matches!(carrier, RegionCarrier::Cylinder { .. });
        if !matches!(carrier, RegionCarrier::Freeform) && !sliver {
            continue;
        }
        let mut walls = BTreeSet::new();
        for &t in triangles {
            for c in 0..3 {
                let (a, b) = (indices[3 * t + c], indices[3 * t + (c + 1) % 3]);
                for &n in &edges[&(a.min(b), a.max(b))] {
                    let r = labels[n];
                    if r != region && cylinder_of(r).is_some() {
                        walls.insert(r);
                    }
                }
            }
        }
        // Each wall's cylinder regions; one each unless crease_walls joins
        // the pieces of one wall.
        let mut classes = Vec::<Vec<u32>>::new();
        for &wall in &walls {
            let Some(cylinder) = cylinder_of(wall) else { continue };
            let joins = |class: &Vec<u32>| {
                crease_walls
                    && class.iter().any(|&w| cylinder_of(w).is_some_and(|c| same_cylinder_wall(c, cylinder, tolerance)))
            };
            let (joined, kept): (Vec<_>, Vec<_>) = std::mem::take(&mut classes).into_iter().partition(joins);
            classes = kept;
            classes.push(std::iter::once(wall).chain(joined.into_iter().flatten()).collect());
        }
        let [class_a, class_b] = &classes[..] else { continue };
        let lead = |class: &[u32]| {
            class.iter().copied().max_by_key(|&w| (members.get(&w).map_or(0, Vec::len), std::cmp::Reverse(w))).unwrap()
        };
        let (wall_a, wall_b) = (lead(class_a), lead(class_b));
        let (Some(first), Some(second)) = (cylinder_of(wall_a), cylinder_of(wall_b)) else { continue };
        let Some((center, normals)) = cylinder_pair_section(first, second, tolerance) else { continue };
        let deviation = class_a.iter().chain(class_b).filter_map(|&w| cylinder_of(w)).map(|c| c.deviation).fold(0.0, f64::max);
        let bar = accepted_radial_bar(tolerance * 4.0, deviation);
        let radial_error = |cylinder: CylinderCarrier, p: Vec3| (radial(cylinder, p).length() - cylinder.radius).abs();
        // The nearest piece of the first wall (on_first) or the second.
        let error = |on_first: bool, p: Vec3| {
            let class = if on_first { class_a } else { class_b };
            class.iter().filter_map(|&w| cylinder_of(w)).map(|c| radial_error(c, p)).fold(f64::INFINITY, f64::min)
        };
        // Each vertex: on the first wall, on the second.
        let mut side = HashMap::<u32, (bool, bool)>::new();
        for &t in triangles {
            for c in 0..3 {
                let v = indices[3 * t + c];
                side.entry(v).or_insert_with(|| (error(true, point(v)) <= bar, error(false, point(v)) <= bar));
            }
        }
        // A stitched T-junction vertex in `chord` sits on the chord of the
        // edge it split and reads neither wall; it takes no side.
        if side.iter().any(|(v, &(a, b))| !a && !b && !chord.contains(v)) {
            continue;
        }
        // Cut only because of crease_walls: a non-freeform region, a wall
        // of several pieces, or a vertex on neither wall.
        let widens = sliver
            || class_a.len() > 1
            || class_b.len() > 1
            || side.values().any(|&(a, b)| !a && !b);
        if widens && triangles.iter().any(|t| forbid.contains(t)) {
            continue;
        }
        let only = |v: u32| match side[&v] {
            (true, false) => Some(true),
            (false, true) => Some(false),
            _ => None,
        };
        let Ok(minor) = first.axis.cross(second.axis).normalized() else { continue };
        let radius = 0.5 * (first.radius + second.radius);
        let mut cuts = BTreeMap::<(u32, u32), Vec3>::new();
        let mut whole = true;
        for &t in triangles {
            for c in 0..3 {
                let (a, b) = (indices[3 * t + c], indices[3 * t + (c + 1) % 3]);
                let key = (a.min(b), a.max(b));
                if cuts.contains_key(&key) || only(a).is_none() || only(b).is_none() || only(a) == only(b) {
                    continue;
                }
                if edges[&key].iter().any(|&n| labels[n] != region) {
                    whole = false;
                    break;
                }
                let (p, q) = (point(key.0), point(key.1));
                let mut found = Vec::new();
                for &normal in &normals {
                    let (sp, sq) = (p.sub(center).dot(normal), q.sub(center).dot(normal));
                    if !(sp * sq < 0.0) {
                        continue;
                    }
                    let x = p.add(q.sub(p).scale(sp / (sp - sq)));
                    let Ok(major) = normal.cross(minor).normalized() else { continue };
                    let along = major.dot(first.axis);
                    let major_radius = radius / (1.0 - along * along).sqrt();
                    let d = x.sub(center);
                    let angle = (d.dot(minor) / radius).atan2(d.dot(major) / major_radius);
                    let on_ellipse = center
                        .add(major.scale(major_radius * angle.cos()))
                        .add(minor.scale(radius * angle.sin()));
                    if on_ellipse.sub(x).length() <= bar {
                        found.push(on_ellipse);
                    }
                }
                let [cut] = found[..] else {
                    whole = false;
                    break;
                };
                cuts.insert(key, cut);
            }
            if !whole {
                break;
            }
        }
        if !whole {
            continue;
        }
        let mut cut_vertex = HashMap::<(u32, u32), u32>::new();
        for (&key, &cut) in &cuts {
            cut_vertex.insert(key, (new_positions.len() / 3) as u32);
            new_positions.extend([cut.x, cut.y, cut.z]);
        }
        for &t in triangles {
            let v = [indices[3 * t], indices[3 * t + 1], indices[3 * t + 2]];
            // The member of the wall this source triangle borders, else the
            // wall's largest.
            let wall_of = |on_first: bool| {
                let (class, lead) = if on_first { (class_a, wall_a) } else { (class_b, wall_b) };
                (0..3)
                    .flat_map(|k| &edges[&(v[k].min(v[(k + 1) % 3]), v[k].max(v[(k + 1) % 3]))])
                    .map(|&n| labels[n])
                    .find(|r| class.contains(r))
                    .unwrap_or(lead)
            };
            let split = |k: usize| cut_vertex.get(&(v[k].min(v[(k + 1) % 3]), v[k].max(v[(k + 1) % 3]))).copied();
            let cut_edges = (0..3).filter(|&k| split(k).is_some()).collect::<Vec<_>>();
            let mut pieces = Vec::<([u32; 3], u32)>::new();
            match cut_edges[..] {
                [] => {
                    let wall = v.iter().find_map(|&x| only(x)).map_or_else(
                        || {
                            let (ea, eb) = v.iter().fold((0.0, 0.0), |(ea, eb), &x| {
                                (ea + error(true, point(x)), eb + error(false, point(x)))
                            });
                            wall_of(ea <= eb)
                        },
                        wall_of,
                    );
                    pieces.push((v, wall));
                }
                [k] => {
                    let m = split(k).unwrap();
                    let (a, b, c) = (v[k], v[(k + 1) % 3], v[(k + 2) % 3]);
                    pieces.push(([a, m, c], wall_of(only(a).unwrap())));
                    pieces.push(([m, b, c], wall_of(only(b).unwrap())));
                }
                [k1, k2] => {
                    // The vertex both cut edges share stands alone.
                    let lone = if (k1 + 1) % 3 == k2 { k2 } else { k1 };
                    let (a, b, c) = (v[lone], v[(lone + 1) % 3], v[(lone + 2) % 3]);
                    let (m1, m2) = (split(lone).unwrap(), split((lone + 2) % 3).unwrap());
                    let (own, other) = (wall_of(only(a).unwrap()), wall_of(only(b).unwrap()));
                    pieces.push(([a, m1, m2], own));
                    pieces.push(([m1, b, c], other));
                    pieces.push(([m1, c, m2], other));
                }
                _ => unreachable!("two sides cut at most two edges of a triangle"),
            }
            for (index, (corners, wall)) in pieces.into_iter().enumerate() {
                let slot = if index == 0 {
                    t
                } else {
                    parent.push(t);
                    labels.push(wall);
                    new_indices.extend([0, 0, 0]);
                    labels.len() - 1
                };
                new_indices[3 * slot..3 * slot + 3].copy_from_slice(&corners);
                labels[slot] = wall;
            }
        }
        cut_any = true;
        if widens {
            widened.push(triangles.clone());
        }
        for class in [class_a, class_b].into_iter().filter(|_| crease_walls) {
            let lowest = class.iter().copied().chain(class.iter().filter_map(|w| joined_walls.get(w).copied())).min().unwrap();
            for (_, to) in joined_walls.iter_mut().filter(|(_, to)| class.contains(to)) {
                *to = lowest;
            }
            for &w in class {
                joined_walls.insert(w, lowest);
            }
        }
    }
    if !cut_any {
        return None;
    }
    // The pieces of one wall meet along mesh edges once the crease between
    // them is cut; one region over them lets split_revolve_pieces part only
    // the pieces that do not touch. Its deviation covers every piece.
    for label in &mut labels {
        if let Some(&to) = joined_walls.get(label) {
            *label = to;
        }
    }
    for (&from, &to) in &joined_walls {
        let deviation = segmentation.regions[from as usize].max_deviation;
        let region = &mut segmentation.regions[to as usize];
        region.max_deviation = region.max_deviation.max(deviation);
    }
    let area = |t: usize| {
        let [a, b, c] = [new_indices[3 * t], new_indices[3 * t + 1], new_indices[3 * t + 2]].map(|v| {
            Vec3::new(new_positions[3 * v as usize], new_positions[3 * v as usize + 1], new_positions[3 * v as usize + 2])
        });
        0.5 * b.sub(a).cross(c.sub(a)).length()
    };
    for region in &mut segmentation.regions {
        region.triangle_count = 0;
        region.area = 0.0;
    }
    for (t, &r) in labels.iter().enumerate() {
        if let Some(region) = segmentation.regions.get_mut(r as usize) {
            region.triangle_count += 1;
            region.area += area(t);
        }
    }
    segmentation.triangle_count = labels.len();
    segmentation.triangle_region_ids = labels;
    crate::hybrid_freeform_split::compact(segmentation);
    Some(CutMesh { positions: new_positions, indices: new_indices, parent, widened })
}

/// Outward normal of a revolve carrier at a point, before its sense (the
/// segmenter's own field, `mesh_segment::segmentation::carrier_normal`).
fn revolve_carrier_normal(carrier: &RegionCarrier, p: Vec3) -> Option<Vec3> {
    match *carrier {
        RegionCarrier::Cylinder { axis_point, axis_dir, .. } => {
            let d = p.sub(axis_point);
            d.sub(axis_dir.scale(d.dot(axis_dir))).normalized().ok()
        }
        RegionCarrier::Cone { apex, axis_dir, half_angle_rad, .. } => {
            let d = p.sub(apex);
            let radial = d.sub(axis_dir.scale(d.dot(axis_dir))).normalized().ok()?;
            let (sin_a, cos_a) = half_angle_rad.sin_cos();
            Some(radial.scale(cos_a).sub(axis_dir.scale(sin_a)))
        }
        RegionCarrier::Sphere { center, .. } => p.sub(center).normalized().ok(),
        RegionCarrier::Torus { center, axis_dir, major_radius, .. } => {
            let d = p.sub(center);
            let radial = d.sub(axis_dir.scale(d.dot(axis_dir))).normalized().ok()?;
            p.sub(center.add(radial.scale(major_radius))).normalized().ok()
        }
        _ => None,
    }
}

/// Distance of a point from a revolve carrier (the segmenter's
/// `carrier_distance`); infinite for anything else.
fn revolve_carrier_distance(carrier: &RegionCarrier, p: Vec3) -> f64 {
    match *carrier {
        RegionCarrier::Cylinder { axis_point, axis_dir, radius, .. } => {
            let d = p.sub(axis_point);
            (d.sub(axis_dir.scale(d.dot(axis_dir))).length() - radius).abs()
        }
        RegionCarrier::Cone { apex, axis_dir, half_angle_rad, .. } => {
            let d = p.sub(apex);
            let s = d.dot(axis_dir);
            let r = d.sub(axis_dir.scale(s)).length();
            ((r - s * half_angle_rad.tan()) * half_angle_rad.cos()).abs()
        }
        RegionCarrier::Sphere { center, radius, .. } => (p.sub(center).length() - radius).abs(),
        RegionCarrier::Torus { center, axis_dir, major_radius, minor_radius, .. } => {
            let d = p.sub(center);
            let s = d.dot(axis_dir);
            let rho = d.sub(axis_dir.scale(s)).length();
            ((rho - major_radius).hypot(s) - minor_radius).abs()
        }
        _ => f64::INFINITY,
    }
}

/// Lane V row 2d ([`HybridSwitches::absorb_revolve`]): a region joins an
/// edge-adjacent revolve region at least eight times its size whose carrier
/// holds every one of its vertices within `accepted_radial_bar(tolerance,
/// host deviation)` and passes the segmenter's own acceptance gate on every
/// non-degenerate triangle: its normal within `normal_tolerance_deg` of the
/// carrier's at the centroid, plus half the spread of the carrier's normals
/// at its three vertices (a chord facet with every vertex on the carrier
/// leans by up to that much: the coarse crossing's one-triangle slivers sit
/// 15-23 degrees off at a 15 degree gate). A cap triangulated from rim
/// vertices alone has every vertex on the wall but faces 90 degrees off. Of
/// several such hosts the one with the smallest largest residual wins
/// (then the lowest id); the host's accepted deviation grows to cover what
/// it took. Smallest fragments go first, in passes repeated until one
/// joins nothing. A host in `forbid` (ids before the join) takes nothing.
/// A vertex in `chord` ([`HybridSwitches::absorb_chord`]) takes no part in
/// the vertex bar. Returns the triangles each host that took fragments ends
/// up holding.
fn absorb_revolve_fragments(
    segmentation: &mut MeshSegmentation,
    vertices: &[Vec3],
    indices: &[u32],
    tolerance: f64,
    diagonal: f64,
    cos_normal_gate: f64,
    sliver_lean: bool,
    chord: &BTreeMap<u32, Vec<(u32, u32)>>,
    forbid: &BTreeSet<u32>,
    share: bool,
) -> BTreeMap<u32, Vec<usize>> {
    let triangle_count = segmentation.triangle_region_ids.len();
    if indices.len() != triangle_count * 3 {
        return BTreeMap::new();
    }
    let corner = |t: usize, c: usize| vertices[indices[3 * t + c] as usize];
    let mut edges = HashMap::<(u32, u32), Vec<usize>>::new();
    for t in 0..triangle_count {
        for c in 0..3 {
            let (a, b) = (indices[3 * t + c], indices[3 * t + (c + 1) % 3]);
            edges.entry((a.min(b), a.max(b))).or_default().push(t);
        }
    }
    let mut members = BTreeMap::<u32, Vec<usize>>::new();
    for (t, &r) in segmentation.triangle_region_ids.iter().enumerate() {
        members.entry(r).or_default().push(t);
    }
    let degenerate = 1.0e-12 * diagonal * diagonal;
    let mut changed = false;
    let mut hosts = BTreeSet::new();
    // Repeat until a pass joins nothing: a sliver visited before the
    // pieces beside it joined their wall sees only those small pieces.
    loop {
        let mut order = members.iter().filter(|(_, t)| !t.is_empty()).map(|(&r, _)| r).collect::<Vec<_>>();
        order.sort_by_key(|r| (members[r].len(), *r));
        let mut joined = false;
        for fragment in order {
            let triangles = members[&fragment].clone();
            if triangles.is_empty() {
                continue;
            }
            let mut neighbours = BTreeSet::new();
            for &t in &triangles {
                for c in 0..3 {
                    let (a, b) = (indices[3 * t + c], indices[3 * t + (c + 1) % 3]);
                    for &n in &edges[&(a.min(b), a.max(b))] {
                        let r = segmentation.triangle_region_ids[n];
                        if r != fragment {
                            neighbours.insert(r);
                        }
                    }
                }
            }
            let mut best: Option<(f64, u32)> = None;
            let mut eligible = Vec::new();
            for host in neighbours {
                let Some(region) = segmentation.regions.get(host as usize) else { continue };
                if forbid.contains(&host) {
                    continue;
                }
                let size = members.get(&host).map_or(0, Vec::len);
                if size < 8 * triangles.len() {
                    continue;
                }
                let bar = accepted_radial_bar(tolerance, region.max_deviation);
                let sense = match region.carrier {
                    RegionCarrier::Cylinder { sense, .. }
                    | RegionCarrier::Cone { sense, .. }
                    | RegionCarrier::Sphere { sense, .. }
                    | RegionCarrier::Torus { sense, .. } => f64::from(sense),
                    _ => continue,
                };
                eligible.push((host, bar, sense));
                let mut worst = 0.0_f64;
                let fit = |t: usize| {
                    let gate = (degenerate, cos_normal_gate, sliver_lean);
                    revolve_triangle_fit(&region.carrier, sense, t, vertices, indices, bar, chord, gate)
                };
                let holds = triangles.iter().all(|&t| {
                    let (residual, facing) = fit(t);
                    worst = worst.max(residual);
                    facing
                });
                if std::env::var("BREP_DEBUG_ROW2D").is_ok() {
                    eprintln!(
                        "[row2d] fragment {fragment} ({} {}, at {:.3},{:.3},{:.3}) host {host} ({size}, {}) worst {worst:.2e} bar {bar:.2e} facing {holds}",
                        triangles.len(),
                        segmentation.regions.get(fragment as usize).map_or("?", |r| r.carrier.kind()),
                        corner(triangles[0], 0).x,
                        corner(triangles[0], 0).y,
                        corner(triangles[0], 0).z,
                        region.carrier.kind()
                    );
                }
                if holds && worst <= bar && best.is_none_or(|(residual, _)| worst < residual) {
                    best = Some((worst, host));
                }
            }
            let Some((residual, host)) = best else {
                // [`HybridSwitches::absorb_share`]: each triangle to the
                // eligible host holding it with the least residual, when
                // every triangle has one.
                if !share || triangles.len() < 2 {
                    continue;
                }
                let gate = (degenerate, cos_normal_gate, sliver_lean);
                let shared: Option<Vec<(usize, u32, f64)>> = triangles
                    .iter()
                    .map(|&t| {
                        eligible
                            .iter()
                            .filter_map(|&(host, bar, sense)| {
                                let carrier = &segmentation.regions[host as usize].carrier;
                                let (residual, facing) =
                                    revolve_triangle_fit(carrier, sense, t, vertices, indices, bar, chord, gate);
                                (facing && residual <= bar).then_some((t, host, residual))
                            })
                            .min_by(|a, b| a.2.total_cmp(&b.2))
                    })
                    .collect();
                let Some(shared) = shared else { continue };
                if std::env::var("BREP_DEBUG_ROW2D").is_ok() {
                    eprintln!("[row2d] fragment {fragment} ({}) shared among {:?}", triangles.len(), shared.iter().map(|s| s.1).collect::<BTreeSet<_>>());
                }
                for (t, host, residual) in shared {
                    segmentation.triangle_region_ids[t] = host;
                    let region = &mut segmentation.regions[host as usize];
                    region.max_deviation = region.max_deviation.max(residual);
                    members.get_mut(&host).unwrap().push(t);
                    hosts.insert(host);
                }
                members.get_mut(&fragment).unwrap().clear();
                joined = true;
                continue;
            };
            for &t in &triangles {
                segmentation.triangle_region_ids[t] = host;
            }
            let region = &mut segmentation.regions[host as usize];
            region.max_deviation = region.max_deviation.max(residual);
            members.get_mut(&host).unwrap().extend(&triangles);
            members.get_mut(&fragment).unwrap().clear();
            hosts.insert(host);
            joined = true;
        }
        if !joined {
            break;
        }
        changed = true;
    }
    if changed {
        crate::hybrid_freeform_split::compact(segmentation);
    }
    hosts
        .into_iter()
        .filter_map(|host| members.remove(&host).filter(|triangles| !triangles.is_empty()).map(|t| (host, t)))
        .collect()
}

/// Row 2d's test of one triangle against a host carrier: the largest
/// distance of its vertices from the carrier (a chord vertex reads nothing
/// of the host when the host holds both ends of its chord: a vertex on a
/// cone's ruling lies between its ruling neighbours too), and whether its
/// facet faces the carrier within the normal gate, plus half the spread of
/// the carrier's normals at its vertices, plus under `sliver_lean` the tilt
/// its vertex residuals can give it across its least altitude. A degenerate
/// facet faces any carrier.
#[allow(clippy::too_many_arguments)]
fn revolve_triangle_fit(
    carrier: &RegionCarrier,
    sense: f64,
    t: usize,
    vertices: &[Vec3],
    indices: &[u32],
    bar: f64,
    chord: &BTreeMap<u32, Vec<(u32, u32)>>,
    (degenerate, cos_normal_gate, sliver_lean): (f64, f64, bool),
) -> (f64, bool) {
    let on_host = |v: u32| revolve_carrier_distance(carrier, vertices[v as usize]) <= bar;
    let chord_of_host =
        |v: u32| chord.get(&v).is_some_and(|spans| spans.iter().any(|&(a, b)| on_host(a) && on_host(b)));
    let corners = [indices[3 * t], indices[3 * t + 1], indices[3 * t + 2]];
    let [a, b, c] = corners.map(|v| vertices[v as usize]);
    let mut worst = 0.0_f64;
    for (&v, p) in corners.iter().zip([a, b, c]) {
        if !chord_of_host(v) {
            worst = worst.max(revolve_carrier_distance(carrier, p));
        }
    }
    let normal = b.sub(a).cross(c.sub(a));
    if normal.length() <= degenerate {
        return (worst, true);
    }
    let angle = |u: Vec3, v: Vec3| u.dot(v).clamp(-1.0, 1.0).acos();
    let (Ok(facet), Some(centroid), Some(na), Some(nb), Some(nc)) = (
        normal.normalized(),
        revolve_carrier_normal(carrier, a.add(b).add(c).scale(1.0 / 3.0)),
        revolve_carrier_normal(carrier, a),
        revolve_carrier_normal(carrier, b),
        revolve_carrier_normal(carrier, c),
    ) else {
        return (worst, false);
    };
    let spread = angle(na, nb).max(angle(nb, nc)).max(angle(nc, na));
    let lean = if sliver_lean {
        let longest = b.sub(a).length().max(c.sub(b).length()).max(a.sub(c).length());
        let residual = [a, b, c].into_iter().map(|p| revolve_carrier_distance(carrier, p)).fold(0.0, f64::max);
        (2.0 * residual * longest / normal.length()).min(1.0).asin().min(cos_normal_gate.acos())
    } else {
        0.0
    };
    (worst, angle(facet.scale(sense), centroid) <= cos_normal_gate.acos() + 0.5 * spread + lean)
}

/// [`HybridSwitches::join_wall_pieces`]: union edge-adjacent cylinder
/// regions that are pieces of one wall into the largest of them (its
/// carrier, the largest deviation, the summed area).
fn join_wall_pieces(segmentation: &mut MeshSegmentation, indices: &[u32], tolerance: f64) {
    let triangle_count = segmentation.triangle_region_ids.len();
    if indices.len() != triangle_count * 3 {
        return;
    }
    let mut edges = HashMap::<(u32, u32), Vec<u32>>::new();
    for (t, corners) in indices.chunks_exact(3).enumerate() {
        for k in 0..3 {
            let (a, b) = (corners[k], corners[(k + 1) % 3]);
            edges.entry((a.min(b), a.max(b))).or_default().push(segmentation.triangle_region_ids[t]);
        }
    }
    let cylinder_of = |r: u32| {
        segmentation.regions.get(r as usize).and_then(|region| carrier_cylinder(region, true).ok().flatten())
    };
    let mut pairs = BTreeSet::new();
    for regions in edges.values() {
        if let [a, b] = regions[..] {
            if a != b {
                pairs.insert((a.min(b), a.max(b)));
            }
        }
    }
    let mut group = BTreeMap::<u32, u32>::new();
    fn root(group: &BTreeMap<u32, u32>, mut r: u32) -> u32 {
        while let Some(&up) = group.get(&r).filter(|&&up| up != r) {
            r = up;
        }
        r
    }
    for (a, b) in pairs {
        let (Some(first), Some(second)) = (cylinder_of(a), cylinder_of(b)) else { continue };
        if !same_cylinder_wall(first, second, tolerance) {
            continue;
        }
        let (ra, rb) = (root(&group, a), root(&group, b));
        if ra != rb {
            group.insert(ra.max(rb), ra.min(rb));
            group.entry(ra.min(rb)).or_insert(ra.min(rb));
        }
    }
    if group.is_empty() {
        return;
    }
    let mut members = BTreeMap::<u32, Vec<u32>>::new();
    for &r in group.keys() {
        members.entry(root(&group, r)).or_default().push(r);
    }
    let mut target = HashMap::<u32, u32>::new();
    for pieces in members.values() {
        let size = |r: u32| segmentation.regions[r as usize].triangle_count;
        let lead = pieces.iter().copied().max_by_key(|&r| (size(r), std::cmp::Reverse(r))).unwrap();
        let deviation = pieces.iter().map(|&r| segmentation.regions[r as usize].max_deviation).fold(0.0, f64::max);
        let area = pieces.iter().map(|&r| segmentation.regions[r as usize].area).sum::<f64>();
        let region = &mut segmentation.regions[lead as usize];
        region.max_deviation = deviation;
        region.area = area;
        for &r in pieces {
            target.insert(r, lead);
        }
    }
    for r in &mut segmentation.triangle_region_ids {
        if let Some(&lead) = target.get(r) {
            *r = lead;
        }
    }
    crate::hybrid_freeform_split::compact(segmentation);
}

/// [`HybridSwitches::cone_refit`]: a cone region past the weld tolerance
/// takes the refit on its vertices other than stitched chord vertices when
/// that refit holds every one of them within the weld tolerance; its
/// deviation is then theirs.
fn refit_chord_cones(segmentation: &mut MeshSegmentation, positions: &[f64], indices: &[u32], weld_tolerance: f64) {
    if indices.len() != segmentation.triangle_region_ids.len() * 3 {
        return;
    }
    let chord = crate::hybrid_whole_freeform::chord_vertices(positions, indices, weld_tolerance);
    let mut vertices = BTreeMap::<u32, BTreeSet<u32>>::new();
    for (t, &r) in segmentation.triangle_region_ids.iter().enumerate() {
        vertices.entry(r).or_default().extend(indices[3 * t..3 * t + 3].iter().filter(|v| !chord.contains(v)));
    }
    for region in &mut segmentation.regions {
        let RegionCarrier::Cone { apex, axis_dir, half_angle_rad, sense } = region.carrier else { continue };
        if region.max_deviation <= weld_tolerance {
            continue;
        }
        let Some(own) = vertices.get(&region.id) else { continue };
        let points: Vec<[f64; 3]> = own
            .iter()
            .map(|&v| {
                let i = 3 * v as usize;
                [positions[i], positions[i + 1], positions[i + 2]]
            })
            .collect();
        let Some((apex, axis, half_angle, deviation)) = crate::hybrid_whole_freeform::refit_cone(
            &points,
            [apex.x, apex.y, apex.z],
            [axis_dir.x, axis_dir.y, axis_dir.z],
            half_angle_rad,
        ) else {
            continue;
        };
        if std::env::var("BREP_DEBUG_SPLIT").is_ok() {
            eprintln!(
                "[cone-refit] region {} tris {} deviation {:.2e} -> {deviation:.2e} on {} vertices (bar {weld_tolerance:.1e})",
                region.id,
                region.triangle_count,
                region.max_deviation,
                points.len()
            );
        }
        if deviation > weld_tolerance {
            continue;
        }
        region.carrier = RegionCarrier::Cone {
            apex: Vec3::new(apex[0], apex[1], apex[2]),
            axis_dir: Vec3::new(axis[0], axis[1], axis[2]),
            half_angle_rad: half_angle,
            sense,
        };
        region.max_deviation = deviation;
    }
}

/// [`HybridSwitches::join_cone_pieces`]: edge-adjacent cone regions whose
/// union refits as one cone ([`crate::hybrid_whole_freeform::refit_cone`]
/// from the larger side's carrier, stitched chord vertices left out) within
/// the pair's radial bar at `tolerance * 4` become one region on that
/// refit, the largest first. A piece's apex is no test: across a narrow
/// sector, tilting the axis trades against the opening, and pieces of one
/// countersink at d0.001 jitter-0.1 hold apexes 1e-3 to 2e-1 apart.
/// [`HybridSwitches::gather_cone_strips`]: the most triangles a region may
/// hold and still be gathered as one strip of a shredded cone.
const GATHER_STRIP_TRIANGLES: usize = 32;

/// [`HybridSwitches::gather_cone_strips`]: every edge-connected run of at
/// least three non-plane regions of at most [`GATHER_STRIP_TRIANGLES`]
/// (whatever the segmenter fitted to each strip: sphere, torus, cone,
/// cylinder or freeform) and at least 64 triangles is fitted with one cone
/// from its own facets (`cone_start`, then `refit_cone` on its vertices);
/// when every vertex holds within the radial bar, the bordering plane
/// strips of at most that size the cone also holds join the run, the cone
/// is fitted again, and the run becomes one cone region.
///
/// With `chord_weld` (under [`HybridSwitches::split_noise_bar`], the weld
/// tolerance), the cone is fitted without the vertices the mesh put on a
/// chord between two of their neighbours (T-junction vertices, which sit
/// inside the cone by that chord's sag): on census record
/// occt_pointed_cone_s1_d0.1_tjunction the 21 strips of the wall refitted at
/// 1.65e-2 against a bar of 6.0e-5 and stayed 327 facets.
fn gather_cone_strips(
    segmentation: &mut MeshSegmentation,
    positions: &[f64],
    indices: &[u32],
    tolerance: f64,
    chord_weld: Option<f64>,
) -> usize {
    let triangle_count = segmentation.triangle_region_ids.len();
    if indices.len() != triangle_count * 3 {
        return 0;
    }
    let chord = chord_weld.map_or_else(BTreeSet::new, |weld| crate::hybrid_whole_freeform::chord_vertices(positions, indices, weld));
    let point = |v: u32| {
        let i = 3 * v as usize;
        [positions[i], positions[i + 1], positions[i + 2]]
    };
    let size = |segmentation: &MeshSegmentation, r: u32| segmentation.regions[r as usize].triangle_count;
    let is_plane = |segmentation: &MeshSegmentation, r: u32| {
        matches!(segmentation.regions[r as usize].carrier, RegionCarrier::Plane { .. })
    };
    let mut neighbours = BTreeMap::<u32, BTreeSet<u32>>::new();
    let mut edges = HashMap::<(u32, u32), Vec<u32>>::new();
    let mut members = BTreeMap::<u32, Vec<usize>>::new();
    for (t, corners) in indices.chunks_exact(3).enumerate() {
        let r = segmentation.triangle_region_ids[t];
        members.entry(r).or_default().push(t);
        for k in 0..3 {
            let (a, b) = (corners[k], corners[(k + 1) % 3]);
            edges.entry((a.min(b), a.max(b))).or_default().push(r);
        }
    }
    for regions in edges.values() {
        if let [a, b] = regions[..] {
            if a != b {
                neighbours.entry(a).or_default().insert(b);
                neighbours.entry(b).or_default().insert(a);
            }
        }
    }
    // A plane strip of the cone (16 triangles on the record) joins the run;
    // a two-triangle plane strip of a coarse bore beside it does not.
    let strip = |segmentation: &MeshSegmentation, r: u32| {
        size(segmentation, r) <= GATHER_STRIP_TRIANGLES && (!is_plane(segmentation, r) || size(segmentation, r) >= 8)
    };
    let mut seen = BTreeSet::new();
    let mut runs = Vec::new();
    for r in 0..segmentation.regions.len() as u32 {
        if seen.contains(&r) || !strip(segmentation, r) || !members.contains_key(&r) {
            continue;
        }
        let mut run = vec![r];
        seen.insert(r);
        let mut at = 0;
        while at < run.len() {
            let here = run[at];
            at += 1;
            for &n in neighbours.get(&here).into_iter().flatten() {
                if !seen.contains(&n) && strip(segmentation, n) {
                    seen.insert(n);
                    run.push(n);
                }
            }
        }
        runs.push(run);
    }
    // A chord vertex the fitted cone does not hold at the bar is dropped
    // and the cone fitted again, at most three times (the mesh's vertices
    // along the rulings pass the chord test too and stay).
    let fit = |regions: &[u32], bar: f64| {
        let triangles: Vec<usize> = regions.iter().flat_map(|r| members[r].iter().copied()).collect();
        let facets: Vec<[[f64; 3]; 3]> = triangles.iter().map(|&t| [0, 1, 2].map(|c| point(indices[3 * t + c]))).collect();
        let mut vertices: Vec<u32> = triangles
            .iter()
            .flat_map(|&t| indices[3 * t..3 * t + 3].iter().copied())
            .collect::<BTreeSet<u32>>()
            .into_iter()
            .collect();
        let (apex, axis, half_angle) = crate::hybrid_whole_freeform::cone_start(&facets)?;
        let mut fitted = crate::hybrid_whole_freeform::refit_cone(&vertices.iter().map(|&v| point(v)).collect::<Vec<_>>(), apex, axis, half_angle)?;
        for _ in 0..3 {
            if chord.is_empty() || fitted.3 <= bar {
                break;
            }
            let (apex, axis, half_angle) = (fitted.0, fitted.1, fitted.2);
            let off = |v: u32| {
                let q = point(v);
                let d = [q[0] - apex[0], q[1] - apex[1], q[2] - apex[2]];
                let h = d[0] * axis[0] + d[1] * axis[1] + d[2] * axis[2];
                let r = ((d[0] - h * axis[0]).powi(2) + (d[1] - h * axis[1]).powi(2) + (d[2] - h * axis[2]).powi(2)).sqrt();
                (r * half_angle.cos() - h * half_angle.sin()).abs() > bar
            };
            let before = vertices.len();
            vertices.retain(|&v| !(chord.contains(&v) && off(v)));
            if vertices.len() == before {
                break;
            }
            fitted = crate::hybrid_whole_freeform::refit_cone(&vertices.iter().map(|&v| point(v)).collect::<Vec<_>>(), apex, axis, half_angle)?;
        }
        Some((fitted, facets))
    };
    let mut gathered = 0;
    let mut claimed = BTreeSet::<u32>::new();
    for mut run in runs {
        let total: usize = run.iter().map(|&r| size(segmentation, r)).sum();
        if run.len() < 3 || total < 64 {
            continue;
        }
        let deviation = run.iter().map(|&r| segmentation.regions[r as usize].max_deviation).fold(0.0, f64::max);
        let bar = accepted_radial_bar(tolerance * 4.0, deviation);
        let Some(((apex, axis, half_angle, refit), _)) = fit(&run, bar) else { continue };
        if !(refit <= bar) {
            if std::env::var("BREP_DEBUG_SPLIT").is_ok() {
                eprintln!("[cone-gather] {} strips ({total} tris): refit {refit:.2e} bar {bar:.2e}, left", run.len());
            }
            continue;
        }
        // Plane strips the cone holds.
        let cone = ConeCarrier {
            apex: Vec3::new(apex[0], apex[1], apex[2]),
            axis: Vec3::new(axis[0], axis[1], axis[2]),
            half_angle,
            sense: 1,
            deviation: refit,
        };
        let holds = |v: u32| {
            let [x, y, z] = point(v);
            let p = Vec3::new(x, y, z);
            chord.contains(&v) || (cone_radial(cone, p).length() * half_angle.cos() - cone_station(cone, p) * half_angle.sin()).abs() <= bar
        };
        let planes: BTreeSet<u32> = run
            .iter()
            .flat_map(|r| neighbours.get(r).into_iter().flatten().copied())
            .filter(|&n| {
                // With `chord_weld`, a bordering cone region of any size the
                // cone holds is its own piece too (census record
                // occt_pointed_cone_s1_d0.1_tjunction: the segmenter's cone
                // of 67 below the 21 strips, fitted apart through a
                // T-junction vertex, met the gathered cone in a ring the
                // builder demoted as not coaxial).
                let piece = (is_plane(segmentation, n) && size(segmentation, n) <= GATHER_STRIP_TRIANGLES)
                    || (chord_weld.is_some()
                        && !claimed.contains(&n)
                        && matches!(segmentation.regions[n as usize].carrier, RegionCarrier::Cone { .. }));
                piece && !run.contains(&n) && members[&n].iter().all(|&t| indices[3 * t..3 * t + 3].iter().all(|&v| holds(v)))
            })
            .collect();
        run.extend(planes);
        let Some(((apex, axis, half_angle, refit), facets)) = fit(&run, bar) else { continue };
        if !(refit <= bar) {
            continue;
        }
        let (apex, axis) = (Vec3::new(apex[0], apex[1], apex[2]), Vec3::new(axis[0], axis[1], axis[2]));
        // The sense from the facets: the side of the outward normal
        // r cos - axis sin they face.
        let facing: f64 = facets
            .iter()
            .map(|t| {
                let [a, b, c] = t.map(|p| Vec3::new(p[0], p[1], p[2]));
                let centroid = a.add(b).add(c).scale(1.0 / 3.0);
                let delta = centroid.sub(apex);
                let radial = delta.sub(axis.scale(delta.dot(axis)));
                let outward = radial
                    .normalized()
                    .map_or(Vec3::new(0.0, 0.0, 0.0), |u| u.scale(half_angle.cos()).sub(axis.scale(half_angle.sin())));
                b.sub(a).cross(c.sub(a)).dot(outward)
            })
            .sum();
        let lead = run.iter().copied().max_by_key(|&r| (size(segmentation, r), std::cmp::Reverse(r))).unwrap();
        if std::env::var("BREP_DEBUG_SPLIT").is_ok() {
            eprintln!("[cone-gather] {} regions into {lead}: refit {refit:.2e} bar {bar:.2e}", run.len());
        }
        claimed.extend(run.iter().copied());
        let area: f64 = run.iter().map(|&r| segmentation.regions[r as usize].area).sum();
        for &r in &run {
            for &t in &members[&r] {
                segmentation.triangle_region_ids[t] = lead;
            }
        }
        let region = &mut segmentation.regions[lead as usize];
        region.carrier = RegionCarrier::Cone {
            apex,
            axis_dir: axis,
            half_angle_rad: half_angle,
            sense: if facing >= 0.0 { 1 } else { -1 },
        };
        region.max_deviation = refit;
        region.area = area;
        gathered += 1;
    }
    if gathered > 0 {
        crate::hybrid_freeform_split::compact(segmentation);
    }
    gathered
}

fn join_cone_pieces(
    segmentation: &mut MeshSegmentation,
    positions: &[f64],
    indices: &[u32],
    tolerance: f64,
    weld_tolerance: f64,
    across: bool,
) {
    let triangle_count = segmentation.triangle_region_ids.len();
    if indices.len() != triangle_count * 3 {
        return;
    }
    let chord = crate::hybrid_whole_freeform::chord_vertices(positions, indices, weld_tolerance);
    let mut edges = HashMap::<(u32, u32), Vec<u32>>::new();
    let mut vertices = BTreeMap::<u32, BTreeSet<u32>>::new();
    for (t, corners) in indices.chunks_exact(3).enumerate() {
        let r = segmentation.triangle_region_ids[t];
        for k in 0..3 {
            let (a, b) = (corners[k], corners[(k + 1) % 3]);
            edges.entry((a.min(b), a.max(b))).or_default().push(r);
        }
        vertices.entry(r).or_default().extend(corners.iter().copied());
    }
    let cone_of = |r: u32| segmentation.regions.get(r as usize).and_then(|region| carrier_cone(region).ok().flatten());
    let mut pairs = BTreeSet::new();
    let mut bordering = BTreeMap::<u32, BTreeSet<u32>>::new();
    for regions in edges.values() {
        if let [a, b] = regions[..] {
            if a != b && cone_of(a).is_some() && cone_of(b).is_some() {
                pairs.insert((a.min(b), a.max(b)));
            }
            if a != b {
                bordering.entry(a).or_default().insert(b);
                bordering.entry(b).or_default().insert(a);
            }
        }
    }
    // [`HybridSwitches::join_cone_across`]: cone pieces bordering one small
    // region between them are candidates too.
    if across {
        for (&bridge, around) in &bordering {
            let small = segmentation.regions.get(bridge as usize).is_some_and(|region| {
                region.triangle_count <= CONE_BRIDGE_TRIANGLES && carrier_cone(region).ok().flatten().is_none()
            });
            if !small {
                continue;
            }
            let cones: Vec<u32> = around.iter().copied().filter(|&r| cone_of(r).is_some()).collect();
            for (i, &a) in cones.iter().enumerate() {
                for &b in &cones[i + 1..] {
                    pairs.insert((a.min(b), a.max(b)));
                }
            }
        }
    }
    if std::env::var("BREP_DEBUG_SPLIT").is_ok() {
        let cones = segmentation.regions.iter().filter(|r| matches!(r.carrier, RegionCarrier::Cone { .. })).count();
        eprintln!("[cone-join] {cones} cone regions, {} candidate pairs (across {across})", pairs.len());
    }
    if pairs.is_empty() && !across {
        return;
    }
    // Each group: its lead, its carrier, its vertices, its deviation bar.
    let size = |r: u32| segmentation.regions[r as usize].triangle_count;
    let mut lead = BTreeMap::<u32, u32>::new();
    let mut groups = BTreeMap::<u32, (ConeCarrier, BTreeSet<u32>, usize, f64)>::new();
    for &(a, b) in &pairs {
        for r in [a, b] {
            if let Some(cone) = cone_of(r) {
                lead.insert(r, r);
                groups.entry(r).or_insert_with(|| (cone, vertices.get(&r).cloned().unwrap_or_default(), size(r), cone.deviation));
            }
        }
    }
    fn root(lead: &BTreeMap<u32, u32>, mut r: u32) -> u32 {
        while let Some(&up) = lead.get(&r).filter(|&&up| up != r) {
            r = up;
        }
        r
    }
    let mut order: Vec<(u32, u32)> = pairs.into_iter().collect();
    order.sort_by_key(|&(a, b)| (std::cmp::Reverse(size(a) + size(b)), a, b));
    let point = |v: u32| {
        let i = 3 * v as usize;
        [positions[i], positions[i + 1], positions[i + 2]]
    };
    let mut joined = false;
    // Under `across`, every cone region at once first: two one-column
    // strips are narrow sectors that leave a cone's axis and opening
    // undetermined, their union together is not.
    if across {
        let cones: Vec<u32> = (0..segmentation.regions.len() as u32).filter(|&r| cone_of(r).is_some()).collect();
        let top = cones.iter().copied().max_by_key(|&r| (size(r), std::cmp::Reverse(r)));
        if let (true, Some(top)) = (cones.len() >= 3, top) {
            let union: BTreeSet<u32> = cones.iter().flat_map(|r| vertices.get(r).into_iter().flatten().copied()).collect();
            let deviation = cones.iter().filter_map(|&r| cone_of(r)).map(|c| c.deviation).fold(0.0, f64::max);
            let bar = accepted_radial_bar(tolerance * 4.0, deviation);
            let cone = cone_of(top).unwrap();
            // A one-column strip's own carrier can be all but flat (half
            // angle 1.564 on census record
            // occt_countersunk_hole_s1_d0.005_jitter-0.1) and the refit
            // diverges from it, so the union starts from its own facets.
            let facets: Vec<[[f64; 3]; 3]> = segmentation
                .triangle_region_ids
                .iter()
                .enumerate()
                .filter(|(_, r)| cones.contains(r))
                .map(|(t, _)| [0, 1, 2].map(|c| point(indices[3 * t + c])))
                .collect();
            let start = crate::hybrid_whole_freeform::cone_start(&facets);
            let refit_on = |keep: &dyn Fn(u32) -> bool| {
                let (apex, axis, half_angle) = start?;
                let points: Vec<[f64; 3]> = union.iter().copied().filter(|&v| keep(v)).map(point).collect();
                crate::hybrid_whole_freeform::refit_cone(&points, apex, axis, half_angle).filter(|fit| fit.3 <= bar)
            };
            let fitted = refit_on(&|_| true).or_else(|| refit_on(&|v| !chord.contains(&v)));
            if std::env::var("BREP_DEBUG_SPLIT").is_ok() {
                eprintln!("[cone-join] all {} cone regions: refit {:?} bar {bar:.2e}", cones.len(), fitted.map(|f| f.3));
            }
            if let Some((apex, axis, half_angle, refit)) = fitted {
                // The fresh start picks its own axis direction, so the sense
                // is read from the facets: the side of the outward normal
                // r cos - axis sin they face.
                let (apex, axis) = (Vec3::new(apex[0], apex[1], apex[2]), Vec3::new(axis[0], axis[1], axis[2]));
                let facing: f64 = facets
                    .iter()
                    .map(|t| {
                        let [a, b, c] = t.map(|p| Vec3::new(p[0], p[1], p[2]));
                        let centroid = a.add(b).add(c).scale(1.0 / 3.0);
                        let delta = centroid.sub(apex);
                        let radial = delta.sub(axis.scale(delta.dot(axis)));
                        let outward = radial.normalized().map_or(Vec3::new(0.0, 0.0, 0.0), |u| {
                            u.scale(half_angle.cos()).sub(axis.scale(half_angle.sin()))
                        });
                        b.sub(a).cross(c.sub(a)).dot(outward)
                    })
                    .sum();
                let carrier = ConeCarrier {
                    apex,
                    axis,
                    half_angle,
                    sense: if facing >= 0.0 { 1 } else { -1 },
                    deviation: refit,
                };
                let count = cones.iter().map(|&r| size(r)).sum();
                for &r in &cones {
                    lead.insert(r, top);
                    groups.remove(&r);
                }
                lead.insert(top, top);
                groups.insert(top, (carrier, union, count, deviation.max(refit)));
                joined = true;
            }
        }
    }
    for (a, b) in order {
        let (ra, rb) = (root(&lead, a), root(&lead, b));
        if ra == rb {
            continue;
        }
        let (big, small) = if (groups[&ra].2, std::cmp::Reverse(ra)) >= (groups[&rb].2, std::cmp::Reverse(rb)) { (ra, rb) } else { (rb, ra) };
        let (cone, own, count, deviation) = &groups[&big];
        let (_, other, other_count, other_deviation) = &groups[&small];
        let union: BTreeSet<u32> = own.union(other).copied().collect();
        let bar = accepted_radial_bar(tolerance * 4.0, deviation.max(*other_deviation));
        let refit_on = |keep: &dyn Fn(u32) -> bool| {
            let points: Vec<[f64; 3]> = union.iter().copied().filter(|&v| keep(v)).map(point).collect();
            crate::hybrid_whole_freeform::refit_cone(
                &points,
                [cone.apex.x, cone.apex.y, cone.apex.z],
                [cone.axis.x, cone.axis.y, cone.axis.z],
                cone.half_angle,
            )
            .filter(|fit| fit.3 <= bar)
        };
        // Stitched chord vertices are left out. Under `across` every vertex
        // is tried first: on a coarse cone nearly every vertex lies between
        // its ruling neighbours, and too few are left to fit.
        let without_chords = |v: u32| !chord.contains(&v);
        let fitted = if across { refit_on(&|_| true).or_else(|| refit_on(&without_chords)) } else { refit_on(&without_chords) };
        if std::env::var("BREP_DEBUG_SPLIT").is_ok() {
            eprintln!("[cone-join] {big} ({count}) + {small} ({other_count}): refit {:?} bar {bar:.2e}", fitted.map(|f| f.3));
        }
        let Some((apex, axis, half_angle, refit)) = fitted else { continue };
        let carrier = ConeCarrier {
            apex: Vec3::new(apex[0], apex[1], apex[2]),
            axis: Vec3::new(axis[0], axis[1], axis[2]),
            half_angle,
            sense: cone.sense,
            deviation: refit,
        };
        let merged = (carrier, union, count + other_count, deviation.max(*other_deviation).max(refit));
        groups.remove(&small);
        groups.insert(big, merged);
        lead.insert(small, big);
        joined = true;
    }
    if !joined {
        return;
    }
    let mut target = HashMap::<u32, u32>::new();
    for &r in lead.keys() {
        let top = root(&lead, r);
        if top != r {
            target.insert(r, top);
        }
    }
    for (&top, (cone, _, _, _)) in &groups {
        if !target.values().any(|&t| t == top) {
            continue;
        }
        let area = target.iter().filter(|(_, &t)| t == top).map(|(&r, _)| segmentation.regions[r as usize].area).sum::<f64>();
        let region = &mut segmentation.regions[top as usize];
        region.carrier = RegionCarrier::Cone { apex: cone.apex, axis_dir: cone.axis, half_angle_rad: cone.half_angle, sense: cone.sense };
        region.max_deviation = cone.deviation;
        region.area += area;
    }
    for r in &mut segmentation.triangle_region_ids {
        if let Some(&top) = target.get(r) {
            *r = top;
        }
    }
    crate::hybrid_freeform_split::compact(segmentation);
}

/// Lane V row 2c ([`HybridSwitches::split_revolve_pieces`]): give every
/// edge-connected piece of a revolve region a region of its own on the same
/// carrier. Triangles that share only a vertex are separate pieces. The
/// first piece (by lowest triangle) keeps the region id; area is shared out
/// by triangle count, as `split_periodic_cylinder` does.
fn split_revolve_pieces(segmentation: &mut MeshSegmentation, indices: &[u32]) {
    let triangle_count = segmentation.triangle_region_ids.len();
    if indices.len() != triangle_count * 3 {
        return;
    }
    let mut edges = HashMap::<(u32, u32), Vec<usize>>::new();
    for t in 0..triangle_count {
        for c in 0..3 {
            let (a, b) = (indices[3 * t + c], indices[3 * t + (c + 1) % 3]);
            edges.entry((a.min(b), a.max(b))).or_default().push(t);
        }
    }
    let mut neighbours = vec![Vec::new(); triangle_count];
    for incident in edges.values() {
        for &x in incident {
            for &y in incident {
                if x != y && segmentation.triangle_region_ids[x] == segmentation.triangle_region_ids[y] {
                    neighbours[x].push(y);
                }
            }
        }
    }
    let revolve = |carrier: &RegionCarrier| {
        matches!(
            carrier,
            RegionCarrier::Cylinder { .. }
                | RegionCarrier::Cone { .. }
                | RegionCarrier::Sphere { .. }
                | RegionCarrier::Torus { .. }
        )
    };
    let mut first_piece = vec![true; segmentation.regions.len()];
    let mut seen = vec![false; triangle_count];
    for start in 0..triangle_count {
        let original = segmentation.triangle_region_ids[start];
        if seen[start] || !segmentation.regions.get(original as usize).is_some_and(|r| revolve(&r.carrier)) {
            continue;
        }
        let mut piece = vec![start];
        seen[start] = true;
        let mut at = 0;
        while at < piece.len() {
            let t = piece[at];
            at += 1;
            for &n in &neighbours[t] {
                if !seen[n] {
                    seen[n] = true;
                    piece.push(n);
                }
            }
        }
        if std::mem::replace(&mut first_piece[original as usize], false) {
            continue;
        }
        let next = segmentation.regions.len() as u32;
        let template = &segmentation.regions[original as usize];
        let mut region = template.clone();
        region.id = next;
        region.triangle_count = piece.len();
        region.area = template.area * piece.len() as f64 / template.triangle_count.max(1) as f64;
        let template = &mut segmentation.regions[original as usize];
        template.triangle_count -= piece.len();
        template.area -= region.area;
        for &t in &piece {
            segmentation.triangle_region_ids[t] = next;
        }
        segmentation.regions.push(region);
    }
}

/// The least-squares plane through `points`: their centroid and the
/// smallest-variance direction of their covariance (Jacobi), unoriented.
fn least_squares_plane(points: &[Vec3]) -> Option<(Vec3, Vec3)> {
    if points.is_empty() {
        return None;
    }
    let inv = 1.0 / points.len() as f64;
    let centroid = points.iter().fold(Vec3::new(0.0, 0.0, 0.0), |acc, &q| acc.add(q.scale(inv)));
    let mut m = [[0.0_f64; 3]; 3];
    for &q in points {
        let d = q.sub(centroid);
        let d = [d.x, d.y, d.z];
        for i in 0..3 {
            for j in 0..3 {
                m[i][j] += d[i] * d[j];
            }
        }
    }
    let mut basis = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    for _ in 0..50 {
        let (mut p, mut q, mut largest) = (0, 1, 0.0_f64);
        for i in 0..3 {
            for j in (i + 1)..3 {
                if m[i][j].abs() > largest {
                    largest = m[i][j].abs();
                    p = i;
                    q = j;
                }
            }
        }
        if largest < 1e-30 {
            break;
        }
        let theta = 0.5 * (m[q][q] - m[p][p]) / m[p][q];
        let t = theta.signum() / (theta.abs() + (theta * theta + 1.0).sqrt());
        let t = if theta == 0.0 { 1.0 } else { t };
        let (c, sn) = (1.0 / (t * t + 1.0).sqrt(), t / (t * t + 1.0).sqrt());
        let mut next = m;
        for k in 0..3 {
            next[k][p] = c * m[k][p] - sn * m[k][q];
            next[k][q] = sn * m[k][p] + c * m[k][q];
        }
        let rotated = next;
        for k in 0..3 {
            next[p][k] = c * rotated[p][k] - sn * rotated[q][k];
            next[q][k] = sn * rotated[p][k] + c * rotated[q][k];
        }
        m = next;
        for row in &mut basis {
            let (a, b) = (row[p], row[q]);
            row[p] = c * a - sn * b;
            row[q] = sn * a + c * b;
        }
    }
    let smallest = (0..3).min_by(|&a, &b| m[a][a].total_cmp(&m[b][b])).unwrap();
    let normal = Vec3::new(basis[0][smallest], basis[1][smallest], basis[2][smallest]).normalized().ok()?;
    Some((centroid, normal))
}

/// [`HybridSwitches::merge_coplanar_planes`]: the fewest triangles each of
/// two planes needs to merge. Smaller plane fragments are the noisy cap
/// join's and the absorb's; on a coarse jittered cylinder wall two adjacent
/// 2-triangle facet strips also hold within the bar (census record
/// occt_stepped_shaft_s10_d0.005_jitter-1.0, whose conversion then failed
/// its STEP round trip).
const COPLANAR_PLANE_TRIANGLES: usize = 16;

/// [`HybridSwitches::merge_coplanar_planes`]: two edge-adjacent plane
/// regions of at least [`COPLANAR_PLANE_TRIANGLES`] each whose vertices all
/// lie within `bar` of their least-squares plane become one region on that
/// plane (oriented as the larger one), the first such pair (by region ids)
/// at a time until none is left.
fn merge_coplanar_planes(segmentation: &mut MeshSegmentation, positions: &[f64], indices: &[u32], bar: f64) {
    let triangle_count = segmentation.triangle_region_ids.len();
    if !(bar > 0.0) || indices.len() != triangle_count * 3 {
        return;
    }
    let point = |v: u32| Vec3::new(positions[3 * v as usize], positions[3 * v as usize + 1], positions[3 * v as usize + 2]);
    let mut edges = HashMap::<(u32, u32), Vec<usize>>::new();
    for t in 0..triangle_count {
        for c in 0..3 {
            let (a, b) = (indices[3 * t + c], indices[3 * t + (c + 1) % 3]);
            edges.entry((a.min(b), a.max(b))).or_default().push(t);
        }
    }
    let plane = |segmentation: &MeshSegmentation, r: u32| match segmentation.regions.get(r as usize).map(|region| &region.carrier) {
        Some(&RegionCarrier::Plane { origin, normal }) => Some((origin, normal)),
        _ => None,
    };
    let mut changed = false;
    loop {
        let mut members = BTreeMap::<u32, BTreeSet<u32>>::new();
        for (t, &r) in segmentation.triangle_region_ids.iter().enumerate() {
            members.entry(r).or_default().extend(&indices[3 * t..3 * t + 3]);
        }
        let mut pairs = BTreeSet::new();
        for incident in edges.values() {
            if let [x, y] = incident.as_slice() {
                let (a, b) = (segmentation.triangle_region_ids[*x], segmentation.triangle_region_ids[*y]);
                if a != b && plane(segmentation, a).is_some() && plane(segmentation, b).is_some() {
                    pairs.insert((a.min(b), a.max(b)));
                }
            }
        }
        let Some((keep, absorb, origin, normal, deviation)) = pairs.into_iter().find_map(|(a, b)| {
            let size = |r: u32| segmentation.regions[r as usize].triangle_count;
            if size(a).min(size(b)) < COPLANAR_PLANE_TRIANGLES {
                return None;
            }
            let (keep, absorb) = if (size(a), std::cmp::Reverse(a)) >= (size(b), std::cmp::Reverse(b)) { (a, b) } else { (b, a) };
            let union = members[&keep].union(&members[&absorb]).map(|&v| point(v)).collect::<Vec<_>>();
            let (origin, normal) = least_squares_plane(&union)?;
            let (_, facing) = plane(segmentation, keep)?;
            let normal = if normal.dot(facing) < 0.0 { normal.scale(-1.0) } else { normal };
            let deviation = union.iter().map(|q| q.sub(origin).dot(normal).abs()).fold(0.0_f64, f64::max);
            (deviation <= bar).then_some((keep, absorb, origin, normal, deviation))
        }) else {
            break;
        };
        if std::env::var("BREP_DEBUG_SPLIT").is_ok() {
            eprintln!(
                "[coplanar] {keep} ({}) + {absorb} ({}): deviation {deviation:.2e} bar {bar:.2e}",
                segmentation.regions[keep as usize].triangle_count,
                segmentation.regions[absorb as usize].triangle_count
            );
        }
        let (count, area) = (segmentation.regions[absorb as usize].triangle_count, segmentation.regions[absorb as usize].area);
        for r in &mut segmentation.triangle_region_ids {
            if *r == absorb {
                *r = keep;
            }
        }
        let region = &mut segmentation.regions[keep as usize];
        region.triangle_count += count;
        region.area += area;
        region.carrier = RegionCarrier::Plane { origin, normal };
        region.max_deviation = deviation;
        segmentation.regions[absorb as usize].triangle_count = 0;
        changed = true;
    }
    if changed {
        crate::hybrid_freeform_split::compact(segmentation);
    }
}

/// Vertex noise the mesh demonstrably carries: the largest residual of a
/// plane region the segmenter accepted with at least 16 triangles (a plane
/// is unambiguous, so its residual is noise, not shape). Zero when no such
/// plane exists.
fn measured_plane_noise(segmentation: &MeshSegmentation) -> f64 {
    segmentation
        .regions
        .iter()
        .filter(|region| matches!(region.carrier, RegionCarrier::Plane { .. }) && region.triangle_count >= 16)
        .map(|region| region.max_deviation)
        .fold(0.0_f64, f64::max)
}

/// [`HybridSwitches::noise_any_region`]: the largest deviation of any
/// analytic region of at least 16 triangles, for a mesh with no such plane.
fn measured_region_noise(segmentation: &MeshSegmentation) -> f64 {
    segmentation
        .regions
        .iter()
        .filter(|region| !matches!(region.carrier, RegionCarrier::Freeform) && region.triangle_count >= 16)
        .map(|region| region.max_deviation)
        .fold(0.0_f64, f64::max)
}

/// On a noisy mesh the small fan triangles of a planar cap tilt past the
/// region grower's dihedral and each piece fails a plane fit whose bar scales
/// with that piece's own size: a jittered frustum cap read as eight freeform
/// and five plane fragments, and the mixed builder demoted the cone wall
/// against them. Edge-connected runs of freeform and small plane regions
/// (under 16 triangles) are joined into one plane region when the
/// least-squares plane of the run holds every vertex within the noise the
/// mesh carries on its large planes (the tilted fan normals that split the
/// cap are not evidence either way). A clean mesh measures no noise and is
/// unchanged.
/// [`HybridSwitches::join_split_runs`]: the cosine of the largest angle
/// between adjacent triangles a split run's piece crosses (30 degrees).
const JOIN_CREASE_COS: f64 = 0.866_025_403_784_438_6;

fn join_noisy_planar_fragments(
    segmentation: &mut MeshSegmentation,
    positions: &[f64],
    indices: &[u32],
    noise: f64,
    curved_slivers: bool,
    split_runs: bool,
    lone_freeform: bool,
) {
    let triangle_count = segmentation.triangle_region_ids.len();
    if !(noise > 0.0) || indices.len() != triangle_count * 3 {
        return;
    }
    // [`HybridSwitches::join_split_runs`]: a split piece can take part of a
    // region, so every region's count must be its true count before the
    // join moves triangles out of it (a stale count underflowed on the
    // assembly fixture of `assembly_retains_curved_faces_and_four_conical_cavities`).
    if split_runs {
        for region in &mut segmentation.regions {
            region.triangle_count = 0;
        }
        for &r in &segmentation.triangle_region_ids {
            if let Some(region) = segmentation.regions.get_mut(r as usize) {
                region.triangle_count += 1;
            }
        }
    }
    let point = |i: u32| {
        let at = i as usize * 3;
        Vec3::new(positions[at], positions[at + 1], positions[at + 2])
    };
    let small = |region: u32| -> bool {
        segmentation.regions.get(region as usize).is_some_and(|r| {
            matches!(r.carrier, RegionCarrier::Freeform)
                || (matches!(r.carrier, RegionCarrier::Plane { .. }) && r.triangle_count < 16)
                // Row 7: once the freeform split has measured the noise, a
                // curved carrier on a handful of cap triangles (a jittered
                // fan read as a 4-triangle cone) is a fragment too; the run
                // still joins only when its plane holds every vertex.
                || (curved_slivers && r.triangle_count < 8)
        })
    };
    let mut edges = HashMap::<(u32, u32), Vec<usize>>::new();
    for t in 0..triangle_count {
        for c in 0..3 {
            let (a, b) = (indices[3 * t + c], indices[3 * t + (c + 1) % 3]);
            edges.entry((a.min(b), a.max(b))).or_default().push(t);
        }
    }
    let mut neighbours = vec![Vec::new(); triangle_count];
    for incident in edges.values() {
        for &x in incident {
            for &y in incident {
                if x != y {
                    neighbours[x].push(y);
                }
            }
        }
    }
    let mut seen = vec![false; triangle_count];
    let mut joins = Vec::new();
    for start in 0..triangle_count {
        if seen[start] || !small(segmentation.triangle_region_ids[start]) {
            continue;
        }
        let mut run = vec![start];
        seen[start] = true;
        let mut at = 0;
        while at < run.len() {
            let t = run[at];
            at += 1;
            for &n in &neighbours[t] {
                if !seen[n] && small(segmentation.triangle_region_ids[n]) {
                    seen[n] = true;
                    run.push(n);
                }
            }
        }
        let fit = |run: &Vec<usize>| {
            let regions = run.iter().map(|&t| segmentation.triangle_region_ids[t]).collect::<BTreeSet<_>>();
            // [`HybridSwitches::join_lone_freeform`]: a run that is one whole
            // freeform region of at least 16 triangles is a cap the kernel
            // segmenter kept whole, and takes the same plane test; anything
            // smaller stays a sliver.
            let lone_cap = lone_freeform
                && regions.len() == 1
                && regions.iter().next().is_some_and(|&r| {
                    segmentation.regions.get(r as usize).is_some_and(|region| {
                        matches!(region.carrier, RegionCarrier::Freeform)
                            && region.triangle_count >= 16
                            && region.triangle_count == run.len()
                    })
                });
            if regions.len() < 2 && !lone_cap {
                return None;
            }
            // Least-squares plane through the run's vertices: the centroid and
            // the smallest-variance direction of their covariance (Jacobi).
            let mut verts = run.iter().flat_map(|&t| (0..3).map(move |c| indices[3 * t + c])).collect::<Vec<_>>();
            verts.sort_unstable();
            verts.dedup();
            let points = verts.iter().map(|&v| point(v)).collect::<Vec<_>>();
            let (centroid, normal) = least_squares_plane(&points)?;
            // Orient with the run's triangles.
            let mut facing = 0.0;
            let mut area = 0.0;
            for &t in run {
                let [a, b, c] = [point(indices[3 * t]), point(indices[3 * t + 1]), point(indices[3 * t + 2])];
                let cross = b.sub(a).cross(c.sub(a));
                facing += cross.dot(normal);
                area += cross.length();
            }
            let normal = if facing < 0.0 { normal.scale(-1.0) } else { normal };
            let max_deviation = verts.iter().map(|&v| point(v).sub(centroid).dot(normal).abs()).fold(0.0_f64, f64::max);
            (max_deviation <= noise).then_some((centroid, normal, max_deviation, area * 0.5))
        };
        if let Some((centroid, normal, max_deviation, area)) = fit(&run) {
            joins.push((run, centroid, normal, max_deviation, area));
            continue;
        }
        // [`HybridSwitches::join_split_runs`]: a run that holds several caps
        // linked through small wall pieces is cut at its creases and each
        // piece is tried alone.
        if !split_runs {
            continue;
        }
        let unit = |t: usize| {
            let [a, b, c] = [point(indices[3 * t]), point(indices[3 * t + 1]), point(indices[3 * t + 2])];
            b.sub(a).cross(c.sub(a)).normalized().ok()
        };
        let members: BTreeSet<usize> = run.iter().copied().collect();
        let mut placed = BTreeSet::new();
        for &seed in &run {
            if !placed.insert(seed) {
                continue;
            }
            let mut piece = vec![seed];
            let mut at = 0;
            while at < piece.len() {
                let t = piece[at];
                at += 1;
                for &n in &neighbours[t] {
                    if members.contains(&n)
                        && !placed.contains(&n)
                        && unit(t).zip(unit(n)).is_some_and(|(a, b)| a.dot(b) >= JOIN_CREASE_COS)
                    {
                        placed.insert(n);
                        piece.push(n);
                    }
                }
            }
            if let Some((centroid, normal, max_deviation, area)) = fit(&piece) {
                joins.push((piece, centroid, normal, max_deviation, area));
            }
        }
    }
    if joins.is_empty() {
        return;
    }
    for (run, origin, normal, max_deviation, area) in joins {
        let mut low = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
        let mut high = Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
        for &t in &run {
            for c in 0..3 {
                let q = point(indices[3 * t + c]);
                low = Vec3::new(low.x.min(q.x), low.y.min(q.y), low.z.min(q.z));
                high = Vec3::new(high.x.max(q.x), high.y.max(q.y), high.z.max(q.z));
            }
        }
        let id = segmentation.regions.len() as u32;
        for &t in &run {
            let old = segmentation.triangle_region_ids[t] as usize;
            segmentation.regions[old].triangle_count -= 1;
            segmentation.triangle_region_ids[t] = id;
        }
        segmentation.regions.push(MeshRegion {
            id,
            triangle_count: run.len(),
            area,
            bbox_min: low,
            bbox_max: high,
            carrier: RegionCarrier::Plane { origin, normal },
            max_deviation,
            rms_deviation: max_deviation,
            max_normal_angle_deg: 30.0,
        });
    }
    // Drop emptied regions and renumber densely.
    let mut remap = vec![u32::MAX; segmentation.regions.len()];
    let mut kept = Vec::new();
    for region in std::mem::take(&mut segmentation.regions) {
        if region.triangle_count == 0 {
            continue;
        }
        remap[region.id as usize] = kept.len() as u32;
        let mut region = region;
        region.id = kept.len() as u32;
        kept.push(region);
    }
    for id in &mut segmentation.triangle_region_ids {
        if *id != brep_kernel::UNASSIGNED_REGION {
            *id = remap[*id as usize];
        }
    }
    segmentation.regions = kept;
}

/// Lane J row 10: a stray region beside the whole primitive has fewer
/// triangles than this (row 7's sliver bar), and the strays together hold at
/// most 1 % of the mesh.
const WHOLE_STRAY_TRIANGLES: usize = 8;

/// [`HybridSwitches::whole_same_kind`]: a stray of another kind beside the
/// pieces of a whole sphere or torus holds fewer triangles than this (the
/// plane count `measured_plane_noise` takes as evidence).
const WHOLE_SAME_KIND_OTHER: usize = 16;

/// [`HybridSwitches::whole_freeform_strays`]: the strays beside a dominant
/// freeform region hold at most one in this many of the mesh's triangles.
const WHOLE_FREEFORM_STRAY_SHARE: usize = 10;

/// Lane J row 10, promoted ON 2026-10-16: BREP_HYBRID_WHOLE_STRAYS=0 restores
/// rows 6 and 9's one-region requirement.
fn whole_strays_enabled() -> bool {
    brep_kernel::switch!(live "BREP_HYBRID_WHOLE_STRAYS", true)
}

/// The exact sphere or ring torus for a closed mesh that the segmentation
/// holds as one region of that carrier, or `None` when any check fails:
/// every triangle in the region, outward sense, a closed edge-manifold mesh
/// of the carrier's genus (0 / 1), and the mesh's enclosed volume within
/// `area × (max vertex deviation + chord sag)` of the primitive's.
fn whole_closed_primitive(
    positions: &[f64],
    indices: &[u32],
    segmentation: &MeshSegmentation,
    weld_tolerance: f64,
    tally: &mut crate::stl_conversion::WholeCarrierScanReport,
    switches: HybridSwitches,
) -> Option<HybridBrepOutput> {
    let triangle_count = indices.len() / 3;
    // Lane J row 10: besides the whole primitive the segmenter may leave stray
    // regions of a few triangles, slivers fitted exactly by any plane or
    // sphere. They join the dominant region; the fit and checks below then run
    // over the whole mesh as before.
    let strays_enabled = whole_strays_enabled();
    let mut whole_fit = false;
    let mut exact_only = false;
    let region = if strays_enabled {
        let region = segmentation.regions.iter().max_by_key(|region| region.triangle_count)?;
        let strays = segmentation.regions.iter().filter(|other| other.id != region.id);
        let admitted = !strays.clone().any(|other| other.triangle_count >= WHOLE_STRAY_TRIANGLES)
            && 100 * strays.clone().map(|other| other.triangle_count).sum::<usize>() <= triangle_count;
        // [`HybridSwitches::whole_same_kind`]: the strays row 10 declines are
        // pieces of the dominant region's sphere or torus kind, freeform
        // regions (no carrier claims them; noise leaves them), and regions
        // of other carriers each under `WHOLE_SAME_KIND_OTHER` triangles (no
        // fit that is evidence) and together at most 2 % of the mesh. The
        // whole fit's residual test, local scan included, decides whether
        // they are noise.
        let kind = region.carrier.kind();
        let revolve = matches!(region.carrier, RegionCarrier::Sphere { .. } | RegionCarrier::Torus { .. });
        let others = strays
            .clone()
            .filter(|other| other.carrier.kind() != kind && !matches!(other.carrier, RegionCarrier::Freeform));
        whole_fit = switches.whole_same_kind
            && !admitted
            && revolve
            && strays.clone().any(|other| other.carrier.kind() == kind && other.triangle_count >= WHOLE_STRAY_TRIANGLES)
            && others.clone().all(|other| other.triangle_count < WHOLE_SAME_KIND_OTHER)
            && 50 * others.map(|other| other.triangle_count).sum::<usize>() <= triangle_count;
        // [`HybridSwitches::whole_exact`]: any strays, an exact fit only.
        exact_only = !whole_fit && switches.whole_exact && !admitted && revolve;
        whole_fit |= exact_only;
        // [`HybridSwitches::whole_freeform_strays`]: beside a dominant
        // freeform region, strays with a carrier each under
        // `WHOLE_SAME_KIND_OTHER` triangles, freeform strays (no carrier,
        // no evidence) each at most `GATHER_STRIP_TRIANGLES`, together at
        // most a tenth of the mesh, are left to the whole fit's residual
        // test (lane J row 9's freeform path).
        let freeform_strays = switches.whole_freeform_strays
            && !admitted
            && !whole_fit
            && matches!(region.carrier, RegionCarrier::Freeform)
            && strays.clone().all(|other| {
                other.triangle_count < WHOLE_SAME_KIND_OTHER
                    || (matches!(other.carrier, RegionCarrier::Freeform) && other.triangle_count <= GATHER_STRIP_TRIANGLES)
            })
            && WHOLE_FREEFORM_STRAY_SHARE * strays.clone().map(|other| other.triangle_count).sum::<usize>() <= triangle_count;
        let admitted = admitted || whole_fit || freeform_strays;
        if segmentation.regions.len() > 1 && std::env::var("BREP_DEBUG_SPLIT").is_ok() {
            let listed: Vec<_> = strays.map(|other| (other.carrier.kind(), other.triangle_count)).collect();
            eprintln!(
                "[stray] {} dominant {} tris {} strays {listed:?}",
                if admitted { "admitted" } else { "declined" },
                region.carrier.kind(),
                region.triangle_count
            );
        }
        if !admitted {
            return None;
        }
        region
    } else {
        let [region] = segmentation.regions.as_slice() else {
            return None;
        };
        region
    };
    let is_stray = |id: u32| id != region.id && segmentation.regions.iter().any(|other| other.id == id);
    if triangle_count == 0
        || segmentation.triangle_region_ids.len() != triangle_count
        || segmentation.triangle_region_ids.iter().any(|&id| id != region.id && !(strays_enabled && is_stray(id)))
    {
        return None;
    }
    let point = |v: u32| {
        let i = 3 * v as usize;
        Vec3::new(positions[i], positions[i + 1], positions[i + 2])
    };
    // Closed, consistently oriented edge incidence; Euler characteristic.
    let mut edges = HashMap::<(u32, u32), i32>::new();
    let mut used = BTreeSet::new();
    let mut longest = 0.0_f64;
    let mut mesh_volume = 0.0;
    for t in indices.chunks_exact(3) {
        if t[0] == t[1] || t[1] == t[2] || t[2] == t[0] {
            return None;
        }
        for k in 0..3 {
            let (a, b) = (t[k], t[(k + 1) % 3]);
            *edges.entry((a.min(b), a.max(b))).or_default() += if a < b { 1 } else { -1 };
            longest = longest.max(point(a).sub(point(b)).length());
            used.insert(a);
        }
        mesh_volume += point(t[0]).dot(point(t[1]).cross(point(t[2]))) / 6.0;
    }
    let chi = used.len() as i64 - edges.len() as i64 + triangle_count as i64;
    if edges.values().any(|&sense| sense != 0) || 2 * edges.len() != 3 * triangle_count {
        return None;
    }
    let mut fitted_deviation = None;
    if switches.whole_refit && strays_enabled && !whole_fit && region.max_deviation > weld_tolerance {
        let genus = match region.carrier {
            RegionCarrier::Sphere { .. } => Some(0),
            RegionCarrier::Torus { .. } => Some(1),
            _ => None,
        };
        let excluded = crate::hybrid_whole_freeform::chord_vertices(positions, indices, weld_tolerance);
        whole_fit = genus.is_some_and(|genus| {
            crate::hybrid_whole_freeform::fit_whole_primitive_without(positions, indices, genus, &excluded)
                .is_some_and(|refit| refit.max_deviation <= weld_tolerance)
        });
    }
    let carrier = if whole_fit { RegionCarrier::Freeform } else { region.carrier.clone() };
    let (solid, genus, area, volume, smallest_radius, sphere) = match carrier {
        RegionCarrier::Sphere { center, radius, sense: 1 } => (
            brep_kernel::make_sphere_brep(center, radius, Vec3::new(0.0, 0.0, 1.0)).ok()?,
            0,
            4.0 * std::f64::consts::PI * radius * radius,
            4.0 / 3.0 * std::f64::consts::PI * radius.powi(3),
            radius,
            true,
        ),
        RegionCarrier::Torus { center, axis_dir, major_radius, minor_radius, sense: 1 }
            if major_radius > minor_radius =>
        (
            brep_kernel::make_torus_brep(center, axis_dir, major_radius, minor_radius).ok()?,
            1,
            4.0 * std::f64::consts::PI * std::f64::consts::PI * major_radius * minor_radius,
            2.0 * std::f64::consts::PI * std::f64::consts::PI * major_radius * minor_radius * minor_radius,
            minor_radius,
            false,
        ),
        // Lane J row 9: one freeform region over the whole closed mesh (no
        // segmenter carrier met its bar under the jitter). The primitive of
        // the mesh's genus is fitted to its vertices and taken only when the
        // residual reads as noise (hybrid_whole_freeform.rs).
        RegionCarrier::Freeform if crate::hybrid_whole_freeform::whole_freeform_enabled() && mesh_volume > 0.0 => {
            let genus = match chi {
                2 => 0,
                0 => 1,
                _ => return None,
            };
            // [`HybridSwitches::whole_same_kind`]: stitched T-junction
            // vertices sit on chords and read nothing of the surface.
            let excluded = if whole_fit {
                crate::hybrid_whole_freeform::chord_vertices(positions, indices, weld_tolerance)
            } else {
                BTreeSet::new()
            };
            let report = crate::hybrid_whole_freeform::fit_whole_primitive_without(positions, indices, genus, &excluded)?;
            if std::env::var("BREP_DEBUG_SPLIT").is_ok() {
                eprintln!(
                    "[whole] genus {genus} vertices {} statistic {:.2} quantile {:.2} local {:.2} threshold {:.2} max deviation {:.3e} {:?}",
                    report.vertices,
                    report.statistic,
                    report.quantile,
                    report.local_statistic,
                    report.local_threshold,
                    report.max_deviation,
                    report.fit
                );
            }
            // On the same-kind path a fit leaving every vertex within the
            // weld tolerance is exact (as row 13 reads it): its residual is
            // roundoff, which the statistics cannot read as noise.
            let exact = whole_fit && report.max_deviation <= weld_tolerance;
            if !exact && (exact_only || !report.residual_is_noise()) {
                return None;
            }
            fitted_deviation = Some(report.max_deviation);
            let v = |p: [f64; 3]| Vec3::new(p[0], p[1], p[2]);
            match report.fit {
                crate::hybrid_whole_freeform::WholeFit::Sphere { center, radius } => (
                    brep_kernel::make_sphere_brep(v(center), radius, Vec3::new(0.0, 0.0, 1.0)).ok()?,
                    0,
                    4.0 * std::f64::consts::PI * radius * radius,
                    4.0 / 3.0 * std::f64::consts::PI * radius.powi(3),
                    radius,
                    true,
                ),
                crate::hybrid_whole_freeform::WholeFit::Torus { center, axis, major, minor } => (
                    brep_kernel::make_torus_brep(v(center), v(axis), major, minor).ok()?,
                    1,
                    4.0 * std::f64::consts::PI * std::f64::consts::PI * major * minor,
                    2.0 * std::f64::consts::PI * std::f64::consts::PI * major * minor * minor,
                    minor,
                    false,
                ),
            }
        }
        _ => return None,
    };
    if chi != 2 - 2 * genus {
        return None;
    }
    // Lane J row 13: the segmentation's sphere or torus carrier passed only
    // the segmenter's own distance bar, which the jitter set; a small flat
    // (h = 0.015 on a 0.006-jitter torus tube) passes it. Row 11's local
    // residual scan reads the carrier's vertex residuals, as row 9's fitted
    // primitive already does; a scan that abstains keeps the status quo. A
    // decline is taken only on a mesh that is not exact: row 9's refit of the
    // primitive must leave a vertex farther off than the weld tolerance (the
    // mesh's own positional resolution). On an exact mesh the carrier's
    // residual is the segmenter's fit error (census r13: 1.7e-4 on a clean
    // coarse torus, whose refit reads roundoff), not noise.
    if fitted_deviation.is_none() && crate::hybrid_whole_freeform::whole_carrier_scan_enabled() {
        let residual = |v: u32| match region.carrier {
            RegionCarrier::Sphere { center, radius, .. } => point(v).sub(center).length() - radius,
            RegionCarrier::Torus { center, axis_dir, major_radius, minor_radius, .. } => {
                let d = point(v).sub(center);
                let axial = d.dot(axis_dir);
                let radial = d.sub(axis_dir.scale(axial)).length();
                (radial - major_radius).hypot(axial) - minor_radius
            }
            _ => f64::NAN,
        };
        let reading = crate::hybrid_whole_freeform::carrier_scan_verdict(indices, residual, if sphere { 4 } else { 7 });
        if std::env::var("BREP_DEBUG_SPLIT").is_ok() {
            eprintln!(
                "[whole-carrier] {} local {:.2} threshold {:.2} independence {:.3} bound {:.3} edges {} {:?}",
                region.carrier.kind(),
                reading.statistic,
                reading.threshold,
                reading.independence,
                reading.bound,
                reading.outside_edges,
                reading.verdict
            );
        }
        let exact = reading.verdict == crate::hybrid_whole_freeform::ScanVerdict::Declined
            && crate::hybrid_whole_freeform::fit_whole_primitive(positions, indices, genus)
                .is_some_and(|refit| refit.max_deviation <= weld_tolerance);
        if !exact {
            tally.add(reading.verdict);
            if reading.verdict == crate::hybrid_whole_freeform::ScanVerdict::Declined {
                return None;
            }
        }
    }
    let sag = longest * longest / (8.0 * smallest_radius);
    if !((mesh_volume - volume).abs() <= area * (fitted_deviation.unwrap_or(region.max_deviation) + sag)) {
        return None;
    }
    if !solid.validate().is_empty() || !solid_signed_volume(&solid).is_ok_and(|v| v > 0.0) {
        return None;
    }
    let mut stats = HybridBrepStats { total_faces: 1, ..Default::default() };
    if sphere {
        stats.analytic_sphere_faces = 1;
        stats.analytic_sphere_triangles = triangle_count;
        stats.segmentation_sphere_regions = 1;
    } else {
        stats.analytic_torus_faces = 1;
        stats.analytic_torus_triangles = triangle_count;
        stats.segmentation_torus_regions = 1;
    }
    Some(HybridBrepOutput { solid, stats })
}

pub(crate) fn hybrid_plane_cylinder_brep_retry(
    positions: &[f64],
    indices: Option<&[u32]>,
    options: &SegmentOptions,
    switches: HybridSwitches,
    injected_failure: Option<u32>,
) -> Result<HybridBrepOutput, String> {
    let (mut best, mut demoted_hosts) =
        rebuild_with_forbidden_hosts(positions, indices, options, switches, injected_failure, &Forbidden::default())?;
    if !(switches.absorb_revolve && switches.absorb_retry) {
        return Ok(best);
    }
    // [`HybridSwitches::absorb_retry`]: each rebuild forbids every host
    // demoted so far; the joins it makes instead can fail in turn.
    // The widened crease cut's regions are forbidden only once no new host
    // is, and in one rebuild more if the hosts used every retry.
    let mut forbid = Forbidden::default();
    let mut seen_cuts = demoted_hosts.cuts.clone();
    for _ in 0..ABSORB_RETRY_LIMIT {
        let before = forbid.hosts.len();
        forbid.hosts.extend(demoted_hosts.hosts);
        if forbid.hosts.len() == before {
            let before = forbid.cuts.len();
            forbid.cuts.extend(demoted_hosts.cuts);
            if forbid.cuts.len() == before {
                break;
            }
        }
        let Ok((output, hosts)) = rebuild_with_forbidden_hosts(positions, indices, options, switches, None, &forbid)
        else {
            break;
        };
        if output.stats.faceted_triangles < best.stats.faceted_triangles {
            best = output;
        }
        seen_cuts.extend(&hosts.cuts);
        demoted_hosts = hosts;
    }
    if !seen_cuts.is_subset(&forbid.cuts) {
        forbid.cuts.extend(seen_cuts);
        if let Ok((output, _)) = rebuild_with_forbidden_hosts(positions, indices, options, switches, None, &forbid) {
            if output.stats.faceted_triangles < best.stats.faceted_triangles {
                best = output;
            }
        }
    }
    Ok(best)
}

/// [`hybrid_brep_with_forbidden_hosts`], made again without
/// [`HybridSwitches::cone_seam`] when it fails with it: a seam that avoids
/// every open trim lets a cone's arcs be built that the arbitrary seam had
/// refused, which demoted only that cone; when such an arc misses its mesh
/// vertices the whole rebuild fails topology validation (census record
/// occt_frustum_s1_d0.05_jitter-0.1, a 5e-3 curve/vertex gap). Each rebuild
/// of the retry falls back on its own: a failed absorb retry was dropped
/// and its gain lost (occt_pointed_cone_s10_d0.02_jitter-0.1 80 -> 88).
/// A rebuild that still fails is made again without
/// [`HybridSwitches::absorb_curved_slivers`] too: slivers joined to two
/// coplanar fragments of one jittered cap left the edge between them 0.04
/// off its vertices (census record occt_cylinder_s10_d0.005_jitter-1.0, which
/// fell to a faceted repair, 131 -> 248 faceted).
fn rebuild_with_forbidden_hosts(
    positions: &[f64],
    indices: Option<&[u32]>,
    options: &SegmentOptions,
    switches: HybridSwitches,
    injected_failure: Option<u32>,
    forbid: &Forbidden,
) -> Result<(HybridBrepOutput, Forbidden), String> {
    let mut rebuilt = hybrid_brep_with_forbidden_hosts(positions, indices, options, switches, injected_failure, forbid);
    let without_seam = HybridSwitches { cone_seam: false, ..switches };
    let without_slivers = HybridSwitches { absorb_curved_slivers: false, ..without_seam };
    // [`HybridSwitches::split_noise_bar`]: a rebuild that still fails is
    // made again without the noise-bar pass, alone and then with the two
    // above: the region goes back to the facets it was. A lone cone the
    // pass proposed on census record occt_pointed_cone_s10_d0.02_jitter-1.0,
    // whose base cap is dust at this jitter, failed topology validation
    // whole (nonzero periodic boundary winding) and the row fell to a
    // faceted repair, 477 -> 496.
    let without_noise_bar = HybridSwitches { split_noise_bar: false, ..switches };
    let without_all = HybridSwitches { split_noise_bar: false, ..without_slivers };
    for (fallback, applies) in [
        (without_seam, switches.cone_seam),
        (without_slivers, switches.absorb_curved_slivers),
        (without_noise_bar, switches.split_noise_bar),
        (without_all, switches.split_noise_bar && (switches.cone_seam || switches.absorb_curved_slivers)),
    ] {
        if rebuilt.is_err() && applies {
            rebuilt = hybrid_brep_with_forbidden_hosts(positions, indices, options, fallback, injected_failure, forbid);
        }
    }
    // [`HybridSwitches::gather_cone_strips`]: a rebuild that gathered a cone
    // is kept only when it leaves fewer faceted triangles than the rebuild
    // without the gather (a gathered countersink at jitter 0.3 was demoted
    // whole, 524 -> 738 faceted, on census record
    // occt_countersunk_hole_s10_d0.005_jitter-0.3).
    // [`HybridSwitches::revolve_ends_first`]: with the revolved chains built
    // first, a vertex disagreement can blame a revolved region where the
    // line built first had blamed a small one (census record
    // occt_countersunk_hole_s1_d0.001_tjunction: 20 -> 3 927 faceted). Only
    // a rebuild that failed or holds such a demotion is made again without
    // the switch, and the one with fewer faceted triangles is kept; every
    // other rebuild pays nothing. A solid whose own STEP the import refuses
    // is the STL conversion's to catch (`hybrid_roundtrip_fallback_enabled`
    // rebuilds it without this switch before the faceted fallback).
    if switches.revolve_ends_first {
        let blamed = rebuilt.as_ref().map_or(true, |(output, _)| {
            output
                .stats
                .demotions
                .iter()
                .any(|demotion| demotion.message.as_deref().is_some_and(|m| m.contains("exact curves disagree")))
        });
        if blamed {
            let without = HybridSwitches { revolve_ends_first: false, ..switches };
            let plain = rebuild_with_forbidden_hosts(positions, indices, options, without, injected_failure, forbid);
            rebuilt = match (rebuilt, plain) {
                (Ok(with), Ok(plain)) if with.0.stats.faceted_triangles < plain.0.stats.faceted_triangles => Ok(with),
                (with, Err(_)) => with,
                (_, plain) => plain,
            };
        }
    }
    // A gathered rebuild with no facet cannot be beaten; only one that still
    // holds a facet is compared with the rebuild without the gather.
    let gathered = rebuilt
        .as_ref()
        .map_or(true, |(output, _)| output.stats.gathered_cone_runs > 0 && output.stats.faceted_triangles > 0);
    if switches.gather_cone_strips && gathered {
        let without = HybridSwitches { gather_cone_strips: false, ..switches };
        let plain = rebuild_with_forbidden_hosts(positions, indices, options, without, injected_failure, forbid);
        rebuilt = match (rebuilt, plain) {
            (Ok(with), Ok(plain)) if with.0.stats.faceted_triangles < plain.0.stats.faceted_triangles => Ok(with),
            (Ok(with), Err(_)) => Ok(with),
            (_, plain) => plain,
        };
    }
    rebuilt
}

/// Rebuilds [`HybridSwitches::absorb_retry`] makes at most.
const ABSORB_RETRY_LIMIT: usize = 4;

/// One hybrid rebuild with its local-demotion retries, row 2d joining no
/// fragment to a host in `forbid`; also the hosts that took fragments and
/// hold a demoted triangle, and the triangles of each region the widened
/// crease cut took whose pieces hold one.
fn hybrid_brep_with_forbidden_hosts(
    positions: &[f64],
    indices: Option<&[u32]>,
    options: &SegmentOptions,
    switches: HybridSwitches,
    mut injected_failure: Option<u32>,
    forbid: &Forbidden,
) -> Result<(HybridBrepOutput, Forbidden), String> {
    // The segmentation does not depend on the demotions; run it once.
    let owned_indices;
    let index_buffer: &[u32] = match indices {
        Some(value) => value,
        None => {
            owned_indices = (0..positions.len() as u32 / 3).collect::<Vec<_>>();
            &owned_indices
        }
    };
    let (mut segmentation, split, absorbed, cut) =
        segment_for_hybrid_measured(positions, index_buffer, options, switches, forbid)?;
    // [`HybridSwitches::crease_cut`]: build on the cut mesh.
    let (positions, index_buffer, parent) = match &cut {
        Some(cut) => (cut.positions.as_slice(), cut.indices.as_slice(), Some(cut.parent.as_slice())),
        None => (positions, index_buffer, None),
    };
    let split_noise = split.noise;
    let mut whole_carrier_scan = crate::stl_conversion::WholeCarrierScanReport::default();
    let with_split = |mut output: HybridBrepOutput| {
        output.stats.split_freeform_regions = split.split_regions;
        output.stats.split_declined_regions = split.declined_regions;
        output.stats.split_over_budget_regions = split.over_budget_regions;
        output.stats.split_budget_hit_regions = split.budget_hit_regions;
        output.stats.split_band_declined_regions = split.band_declined_regions;
        output.stats.gathered_cone_runs = split.gathered_cone_runs;
        output
    };
    // Lane J row 6, promoted ON: a closed mesh the segmentation holds as ONE
    // sphere or ring-torus region has no boundary ring for the builder below
    // (it failed "not bounded by closed rings" and faceted everything), while
    // raw RANSAC fragmented the same noisy body and never offered the whole
    // primitive. BREP_HYBRID_WHOLE_PRIMITIVE=0 restores the faceted path.
    if brep_kernel::switch!(live "BREP_HYBRID_WHOLE_PRIMITIVE", true) {
        if let Some(mut output) = whole_closed_primitive(
            positions,
            index_buffer,
            &segmentation,
            options.weld_tolerance,
            &mut whole_carrier_scan,
            switches,
        ) {
            output.stats.whole_carrier_scan = whole_carrier_scan;
            return Ok((with_split(output), Forbidden::default()));
        }
    }
    let mut split_attempted = BTreeSet::new();
    // Start with empty topology. Building, validating and tessellating a full
    // faceted solid only to clear it costs work proportional to every source
    // triangle and leaves large slot-map storage to clone on each retry.
    // The builder below checks welded vertex identity, triangle bounds and
    // closed, oppositely oriented edge incidence directly. Final topology and
    // geometry still pass the ordinary reconstruction validation.
    let blank = TopologyArena::from_brep(&BrepSolid {
        mass_properties_cache: Default::default(),
        cells: Default::default(),
        id: 1,
        genus: 0,
        kind: brep_kernel::BodyKind::Solid,
        vertices: Vec::new(),
        edges: Vec::new(),
        shells: Vec::new(),
    })?;
    let mut forced_demotions = BTreeMap::<u32, (DemotionCause, String)>::new();
    loop {
        let attempt = if let Some(original) = injected_failure.take() {
            Err(HybridBuildFailure::Region {
                original,
                message: "injected localized carrier construction failure".into(),
                cause: DemotionCause::Injected,
            })
        } else {
            hybrid_plane_cylinder_brep_once(
                positions,
                index_buffer,
                options,
                &segmentation,
                split_noise,
                blank.clone(),
                &forced_demotions,
                switches,
                parent,
            )
        };
        match attempt {
            Ok(mut output) => {
                output.stats.whole_carrier_scan = whole_carrier_scan;
                let demoted = output.stats.demotions.iter().map(|d| d.region).collect::<BTreeSet<_>>();
                let hosts = absorbed
                    .iter()
                    .filter(|(_, triangles)| {
                        triangles.iter().any(|&t| demoted.contains(&segmentation.triangle_region_ids[t]))
                    })
                    .map(|(&host, _)| host)
                    .collect();
                // A widened cut region's first pieces keep their source
                // triangle's index.
                let cuts = cut
                    .iter()
                    .flat_map(|cut| &cut.widened)
                    .filter(|triangles| {
                        triangles.iter().any(|&t| demoted.contains(&segmentation.triangle_region_ids[t]))
                    })
                    .flatten()
                    .copied()
                    .collect();
                return Ok((with_split(output), Forbidden { hosts, cuts }));
            }
            Err(HybridBuildFailure::Periodic { original }) => {
                if split_attempted.len() < HYBRID_RETRY_LIMIT
                    && split_attempted.insert(original)
                    && split_periodic_cylinder(&mut segmentation, original, positions, index_buffer)
                {
                    continue;
                }
                if forced_demotions
                    .insert(original, (DemotionCause::PeriodicSeam, "no safe periodic split".into()))
                    .is_some()
                    || forced_demotions.len() > HYBRID_RETRY_LIMIT
                {
                    return Err(format!("hybrid BREP: no safe periodic split for region {original}"));
                }
            }
            Err(HybridBuildFailure::Region { original, message, cause }) => {
                if std::env::var("BREP_DEBUG_HYBRID").is_ok() {
                    eprintln!("[hybrid] retry: region {original} failed: {message}");
                }
                if forced_demotions.insert(original, (cause, message.clone())).is_some() {
                    return Err(format!(
                        "hybrid BREP: region {original} remained unconstructible after local demotion: {message}"
                    ));
                }
                if forced_demotions.len() > HYBRID_RETRY_LIMIT {
                    return Err(format!(
                        "hybrid BREP: more than {HYBRID_RETRY_LIMIT} regions needed local demotion; last: region {original}: {message}"
                    ));
                }
            }
            Err(HybridBuildFailure::Speculative { originals, message }) => {
                if std::env::var("BREP_DEBUG_HYBRID").is_ok() {
                    eprintln!("[hybrid] retry: fitted-pcurve regions {originals:?} failed: {message}");
                }
                let before = forced_demotions.len();
                for original in originals {
                    forced_demotions
                        .entry(original)
                        .or_insert_with(|| (DemotionCause::WholeBodyCheck, message.clone()));
                }
                if forced_demotions.len() == before {
                    return Err(message);
                }
                if forced_demotions.len() > HYBRID_RETRY_LIMIT {
                    return Err(format!(
                        "hybrid BREP: more than {HYBRID_RETRY_LIMIT} regions needed local demotion; last: {message}"
                    ));
                }
            }
            Err(HybridBuildFailure::Global(message)) => return Err(message),
        }
    }
}

/// Split a periodic wall only along existing axial mesh edges. No triangle
/// is cut or omitted, and both patches keep the same fitted carrier.
fn split_periodic_cylinder(
    segmentation: &mut MeshSegmentation,
    original: u32,
    positions: &[f64],
    indices: &[u32],
) -> bool {
    let template = segmentation.regions[original as usize].clone();
    let RegionCarrier::Cylinder {
        axis_point,
        axis_dir,
        radius,
        ..
    } = template.carrier
    else {
        return false;
    };
    let point = |i: u32| {
        Vec3::new(
            positions[i as usize * 3],
            positions[i as usize * 3 + 1],
            positions[i as usize * 3 + 2],
        )
    };
    let radial = |p: Vec3| {
        let d = p.sub(axis_point);
        d.sub(axis_dir.scale(d.dot(axis_dir)))
    };
    let triangles = segmentation
        .triangle_region_ids
        .iter()
        .enumerate()
        .filter_map(|(t, &r)| (r == original).then_some(t))
        .collect::<Vec<_>>();
    let Some(&first) = triangles.first() else {
        return false;
    };
    let Ok(x) = radial(point(indices[first * 3])).normalized() else {
        return false;
    };
    let y = axis_dir.cross(x);
    let sides = triangles
        .iter()
        .map(|&t| {
            let center = (0..3)
                .map(|c| point(indices[t * 3 + c]))
                .fold(Vec3::new(0.0, 0.0, 0.0), Vec3::add)
                .scale(1.0 / 3.0);
            radial(center).dot(y) > 0.0
        })
        .collect::<Vec<_>>();
    if sides.iter().all(|&s| s == sides[0]) {
        return false;
    }
    // Check every triangle stays on one side except vertices on the dividing
    // meridians. This refuses a split through a facet or an oblique mesh edge.
    let bar = (radius * 1.0e-7).max(template.max_deviation * 2.0);
    if triangles.iter().zip(&sides).any(|(&t, &side)| {
        (0..3).any(|c| {
            let d = radial(point(indices[t * 3 + c])).dot(y);
            if side {
                d < -bar
            } else {
                d > bar
            }
        })
    }) {
        return false;
    }
    let next = segmentation.regions.len() as u32;
    let mut second = template.clone();
    second.id = next;
    second.triangle_count = sides.iter().filter(|&&s| s).count();
    second.area *= second.triangle_count as f64 / template.triangle_count as f64;
    let first = &mut segmentation.regions[original as usize];
    first.triangle_count -= second.triangle_count;
    first.area -= second.area;
    for (&t, &side) in triangles.iter().zip(&sides) {
        if side {
            segmentation.triangle_region_ids[t] = next;
        }
    }
    segmentation.regions.push(second);
    true
}

fn hybrid_plane_cylinder_brep_once(
    positions: &[f64],
    indices: &[u32],
    options: &SegmentOptions,
    segmentation: &MeshSegmentation,
    split_noise: f64,
    mut arena: TopologyArena,
    forced_demotions: &BTreeMap<u32, (DemotionCause, String)>,
    switches: HybridSwitches,
    parent: Option<&[usize]>,
) -> Result<HybridBrepOutput, HybridBuildFailure> {
    if positions.is_empty()
        || !positions.len().is_multiple_of(3)
        || !indices.len().is_multiple_of(3)
    {
        return Err("hybrid BREP: invalid triangle buffers".into());
    }
    let mut vertices = positions
        .chunks_exact(3)
        .map(|p| Vec3::new(p[0], p[1], p[2]))
        .collect::<Vec<_>>();
    let triangles = indices
        .chunks_exact(3)
        .map(|t| [t[0] as usize, t[1] as usize, t[2] as usize])
        .collect::<Vec<_>>();
    if triangles
        .iter()
        .flatten()
        .any(|&vertex| vertex >= vertices.len())
    {
        return Err("hybrid BREP: triangle index outside vertex buffer".into());
    }

    if segmentation.welded_vertex_count != vertices.len() {
        return Err(format!(
            "hybrid BREP: segmentation welded {} input vertices to {}; pre-weld the mesh before exact reconstruction",
            vertices.len(), segmentation.welded_vertex_count
        ).into());
    }
    if segmentation.triangle_region_ids.len() != triangles.len() {
        return Err("hybrid BREP: segmentation triangle count drift".into());
    }

    let diagonal = mesh_diagonal(&vertices);
    // Exact shared topology must be substantially tighter than carrier
    // recognition. This still covers deterministic f32 STL quantization.
    let tolerance = (options.fit_tolerance * diagonal)
        .min(2.0e-6 * diagonal)
        .max(1.0e-10);
    // Lane J row 5, promoted ON: the exact-topology bar is raised to twice
    // the vertex noise the mesh carries on its large planes.
    // BREP_HYBRID_MESH_NOISE=0 keeps the bar at the fit tolerance alone.
    let tolerance = if brep_kernel::switch!(live "BREP_HYBRID_MESH_NOISE", true) {
        tolerance.max(2.0 * measured_plane_noise(segmentation).max(split_noise))
    } else {
        tolerance
    };
    let incidence = edge_incidence(&triangles)?;
    validate_closed_incidence(&triangles, &incidence)?;
    let local = local_regions(
        &segmentation.regions,
        &segmentation.triangle_region_ids,
        &triangles,
        &vertices,
        &incidence,
        tolerance,
        forced_demotions,
        switches,
        parent,
    )?;
    let stats = local.stats;
    let local_to_original = local.local_to_original;
    let planes = local.planes;
    let mut cylinder_carriers = local.cylinders;
    let mut cone_carriers = local.cones;
    let sphere_carriers = local.spheres;
    let torus_carriers = local.tori;
    let mut region_centroids = HashMap::<u32, (Vec3, usize)>::new();
    let mut region_samples = HashMap::<u32, Vec<Vec3>>::new();
    for (triangle, &region) in local.triangle_regions.iter().enumerate() {
        if !sphere_carriers.contains_key(&region) && !torus_carriers.contains_key(&region) {
            continue;
        }
        let [a, b, c] = triangles[triangle];
        let centroid = vertices[a].add(vertices[b]).add(vertices[c]).scale(1.0 / 3.0);
        region_samples.entry(region).or_default().push(centroid);
        let entry = region_centroids.entry(region).or_insert((Vec3::new(0.0, 0.0, 0.0), 0));
        entry.0 = entry.0.add(centroid);
        entry.1 += 1;
    }
    let region_centroids = region_centroids
        .into_iter()
        .map(|(region, (sum, count))| (region, sum.scale(1.0 / count as f64)))
        .collect::<HashMap<_, _>>();
    stage_trace("local regions done");
    let (chains, mut boundaries) =
        region_boundaries(&triangles, &local.triangle_regions, local.count, &incidence)?;
    let is_revolve = |region: u32| {
        cylinder_carriers.contains_key(&region)
            || cone_carriers.contains_key(&region)
            || sphere_carriers.contains_key(&region)
            || torus_carriers.contains_key(&region)
    };
    let chains = split_bent_planar_chains(chains, &mut boundaries, &vertices, tolerance, is_revolve);
    stage_trace("boundaries done");

    snap_cylinder_axes(
        &mut cylinder_carriers,
        &planes,
        &chains,
        &vertices,
        tolerance,
    )
    .map_err(|(region, message)| HybridBuildFailure::Region {
        original: local_to_original[&region],
        message,
        cause: DemotionCause::CylinderAxisSnap,
    })?;
    if switches.shared_crossing {
        share_crossing_wall_carriers(&mut cylinder_carriers, &chains, tolerance);
    }
    snap_cone_carriers(
        &mut cone_carriers,
        &cylinder_carriers,
        &planes,
        &chains,
        &vertices,
        tolerance,
        switches.pointed_cone_apex,
        switches.frustum_two_rings,
    )
    .map_err(|(region, message)| HybridBuildFailure::Region {
        original: local_to_original[&region],
        message,
        cause: DemotionCause::ConeSnap,
    })?;
    snap_cylinder_ruling_vertices(
        &mut vertices,
        &chains,
        &planes,
        &cylinder_carriers,
        tolerance,
    );
    let chains = split_crossing_cylinder_chains(
        chains,
        &mut boundaries,
        &mut vertices,
        &cylinder_carriers,
        tolerance,
    );
    if split_freeform_enabled() {
        pin_crossing_chain_ends(&chains, &mut vertices, &cylinder_carriers, tolerance);
    }
    if switches.pointed_cone_apex {
        apex_at_vertex(&mut cone_carriers, &local.triangle_regions, &triangles, &vertices, tolerance);
    }
    let pointed_cones = pointed_cone_regions(
        &cone_carriers,
        &local.triangle_regions,
        &triangles,
        &vertices,
        &chains,
        tolerance,
    );
    for (&region, cone) in &cone_carriers {
        if pointed_cones.contains(&region) {
            continue;
        }
        let mut low = f64::INFINITY;
        let mut high = f64::NEG_INFINITY;
        for chain in chains
            .iter()
            .filter(|chain| chain.adjacent.0 == region || chain.adjacent.1 == region)
        {
            for point in chain_points(chain, &vertices) {
                let station = cone_station(*cone, point);
                low = low.min(station);
                high = high.max(station);
            }
        }
        validate_cone_span(
            local_to_original[&region],
            low,
            high,
            cone.half_angle,
            tolerance,
        )?;
    }
    stage_trace("axes snapped");
    let cylinders = build_cylinder_surfaces(&cylinder_carriers, &chains, &vertices, tolerance)
        .map_err(|(region, message)| match message {
            Some(message) => HybridBuildFailure::Region {
                original: local_to_original[&region],
                message,
                cause: DemotionCause::CylinderSurface,
            },
            None => HybridBuildFailure::Periodic { original: local_to_original[&region] },
        })?;
    let cones = build_cone_surfaces(
        &cone_carriers,
        &pointed_cones,
        &cylinders,
        &chains,
        &vertices,
        tolerance,
        switches.cone_seam,
        switches.gather_cone_strips,
    )
        .map_err(|(region, message)| HybridBuildFailure::Region {
            original: local_to_original[&region],
            message,
        cause: DemotionCause::ConeSurface,
        })?;
    let spheres = build_sphere_surfaces(
        &sphere_carriers,
        &planes,
        &cylinders,
        &cones,
        &chains,
        &vertices,
        &region_centroids,
        &region_samples,
        tolerance,
    )
    .map_err(|(region, message)| HybridBuildFailure::Region {
        original: local_to_original[&region],
        message,
        cause: DemotionCause::SphereSurface,
    })?;
    let tori = build_torus_surfaces(
        &torus_carriers,
        &cylinders,
        &cones,
        &chains,
        &vertices,
        &region_samples,
        tolerance,
    )
    .map_err(|(region, message)| HybridBuildFailure::Region {
        original: local_to_original[&region],
        message,
        cause: DemotionCause::TorusSurface,
    })?;
    stage_trace("surfaces built");
    sort_region_cycles(
        &mut boundaries,
        &planes,
        &cylinders,
        &cones,
        &spheres,
        &tori,
        &chains,
        &vertices,
    );
    stage_trace("cycles sorted");

    if switches.plane_corners {
        place_plane_corners(&mut vertices, &chains, &planes, tolerance);
    }
    let mut ids = Ids(2);
    let mut arena_vertices = HashMap::new();
    // The regions of the chain that first placed each endpoint vertex.  A
    // line between two planes can meet an arc whose exact end disagrees
    // with the mesh vertex (a revolved region that absorbed chord points
    // where a boolean cut its tessellation); the line itself has no region
    // to demote, so the blame goes to the revolve that placed the vertex.
    let mut vertex_placers = HashMap::<usize, (u32, u32)>::new();

    let mut built_slots: Vec<Option<BuiltEdge>> = (0..chains.len()).map(|_| None).collect();
    let chain_trace = std::env::var("BREP_DEBUG_HYBRID").is_ok();
    let revolves = |chain: &Chain| {
        [chain.adjacent.0, chain.adjacent.1].iter().any(|region| {
            cylinders.contains_key(region)
                || cones.contains_key(region)
                || spheres.contains_key(region)
                || tori.contains_key(region)
        })
    };
    // [`HybridSwitches::revolve_ends_first`]: chains on a revolved region
    // first, so their exact ends place the vertices the lines then join.
    let mut order: Vec<usize> = (0..chains.len()).collect();
    let mut common_station = HashMap::<usize, f64>::new();
    let mut moved_line_ends = 0usize;
    if switches.revolve_ends_first {
        order.sort_by_key(|&index| !revolves(&chains[index]));
        common_station = common_arc_stations(&chains, &vertices, &cylinders, &cones, tolerance);
    }
    for chain_index in order {
        let chain = &chains[chain_index];
        let involves_revolve = revolves(chain);
        if chain_trace && involves_revolve {
            let kind = |region: u32| {
                if planes.contains_key(&region) {
                    "plane"
                } else if cylinders.contains_key(&region) {
                    "cylinder"
                } else if cones.contains_key(&region) {
                    "cone"
                } else if spheres.contains_key(&region) {
                    "sphere"
                } else if tori.contains_key(&region) {
                    "torus"
                } else {
                    "?"
                }
            };
            eprintln!(
                "[hybrid] chain {chain_index}: {} {} / {} {}, {} points, closed={}",
                chain.adjacent.0,
                kind(chain.adjacent.0),
                chain.adjacent.1,
                kind(chain.adjacent.1),
                chain.vertices.len(),
                chain.closed
            );
        }
        let built = match build_chain_edge(
            chain_index,
            chain,
            &vertices,
            &planes,
            &cylinders,
            &cones,
            &spheres,
            &tori,
            tolerance,
            switches,
            &common_station,
            &mut moved_line_ends,
            &mut arena,
            &mut arena_vertices,
            &mut ids,
        ) {
            Ok(built) => built,
            Err(message) => {
                let local = [chain.adjacent.0, chain.adjacent.1]
                    .into_iter()
                    .find(|region| spheres.contains_key(region) || tori.contains_key(region))
                    .map(|region| (region, DemotionCause::ChainEdgeCurved))
                    .or_else(|| {
                        [chain.adjacent.0, chain.adjacent.1]
                            .into_iter()
                            .find(|region| cones.contains_key(region))
                            .map(|region| (region, DemotionCause::ChainEdgeCone))
                    })
                    .or_else(|| {
                        [chain.adjacent.0, chain.adjacent.1]
                            .into_iter()
                            .find(|region| cylinders.contains_key(region))
                            .map(|region| (region, DemotionCause::ChainEdgeCylinder))
                    });
                let local = local.or_else(|| {
                    let endpoints = [chain.vertices.first(), chain.vertices.last()];
                    endpoints
                        .into_iter()
                        .flatten()
                        .filter_map(|vertex| vertex_placers.get(vertex))
                        .flat_map(|&(a, b)| [a, b])
                        .find(|region| {
                            cylinders.contains_key(region)
                                || cones.contains_key(region)
                                || spheres.contains_key(region)
                                || tori.contains_key(region)
                        })
                        .map(|region| (region, DemotionCause::ChainEdgeVertexPlacer))
                });
                if let Some((local, cause)) = local {
                    return Err(HybridBuildFailure::Region {
                        original: local_to_original[&local],
                        message,
                        cause,
                    });
                }
                return Err(HybridBuildFailure::Global(message));
            }
        };
        for vertex in [chain.vertices.first(), chain.vertices.last()].into_iter().flatten() {
            vertex_placers.entry(*vertex).or_insert(chain.adjacent);
        }
        built_slots[chain_index] = Some(built);
    }
    let built_edges: Vec<BuiltEdge> = built_slots.into_iter().map(|built| built.expect("every chain is built")).collect();

    stage_trace("chain edges built");
    let mut faces = Vec::<FaceId>::new();
    for region in 0..local.count as u32 {
        let face = if let Some(plane) = planes.get(&region) {
            build_plane_face(
                region,
                *plane,
                &boundaries[region as usize],
                &chains,
                &built_edges,
                &vertices,
                &mut arena,
                &mut ids,
            )?
        } else if let Some(cylinder) = cylinders.get(&region) {
            build_cylinder_face(
                region,
                cylinder,
                &boundaries[region as usize],
                &built_edges,
                &mut arena,
                &mut ids,
            )
            .map_err(|message| HybridBuildFailure::Region {
                original: local_to_original[&region],
                message,
        cause: DemotionCause::CylinderFace,
            })?
        } else if let Some(torus) = tori.get(&region) {
            build_revolve_band_face(
                region,
                "torus",
                &torus.surface,
                torus.carrier.sense == 1,
                &boundaries[region as usize],
                &built_edges,
                &mut arena,
                &mut ids,
            )
            .map_err(|message| HybridBuildFailure::Region {
                original: local_to_original[&region],
                message,
        cause: DemotionCause::TorusFace,
            })?
        } else if let Some(sphere) = spheres.get(&region) {
            build_sphere_face(
                region,
                sphere,
                &boundaries[region as usize],
                &built_edges,
                &mut arena,
                &mut ids,
            )
            .map_err(|message| HybridBuildFailure::Region {
                original: local_to_original[&region],
                message,
        cause: DemotionCause::SphereFace,
            })?
        } else {
            build_cone_face(
                region,
                &cones[&region],
                &boundaries[region as usize],
                &built_edges,
                &mut arena,
                &mut ids,
            )
            .map_err(|message| HybridBuildFailure::Region {
                original: local_to_original[&region],
                message,
        cause: DemotionCause::ConeFace,
            })?
        };
        faces.push(face);
    }
    if std::env::var("BREP_DEBUG_HYBRID").is_ok() {
        for (region, &face) in faces.iter().enumerate() {
            let record = &arena.faces[face];
            let kind = if planes.contains_key(&(region as u32)) {
                "plane"
            } else if cylinders.contains_key(&(region as u32)) {
                "cylinder"
            } else if cones.contains_key(&(region as u32)) {
                "cone"
            } else if tori.contains_key(&(region as u32)) {
                "torus"
            } else {
                "sphere"
            };
            let coedges = record
                .loops
                .iter()
                .flat_map(|&l| arena.loops[l].coedges.iter())
                .map(|&c| {
                    let coedge = &arena.coedges[c];
                    let edge = &arena.edges[coedge.edge];
                    format!("c{}:e{}{}", coedge.wire_id, edge.wire_id, if edge.degenerate { "*" } else { "" })
                })
                .collect::<Vec<_>>()
                .join(" ");
            eprintln!(
                "[hybrid] face {} = region {region} ({kind}, original {}): {coedges}",
                record.wire_id,
                local_to_original.get(&(region as u32)).copied().unwrap_or(u32::MAX)
            );
        }
    }
    arena.shells.insert(ArenaShell {
        wire_id: ids.next(),
        faces,
    });
    stage_trace("faces built");
    let speculative = built_edges
        .iter()
        .flat_map(|built| built.custom_revolve_pcurves.keys())
        .filter(|region| spheres.contains_key(region) || cylinders.contains_key(region))
        .map(|region| local_to_original[region])
        .collect::<BTreeSet<_>>();
    let fail = |message: String| {
        if speculative.is_empty() {
            HybridBuildFailure::Global(message)
        } else {
            HybridBuildFailure::Speculative {
                originals: speculative.iter().copied().collect(),
                message,
            }
        }
    };
    let mut stats = stats;
    stats.moved_line_ends = moved_line_ends;
    let output = finish_shell(arena, stats).map_err(&fail)?;
    // The rebuild must keep the source shell's topology. A closed source
    // mesh has genus (2 − V + E − F)/2; an output with another genus is a
    // different body however close its area and volume: the kernel's
    // crossed cylinders at chord 0.05 rebuilt with exact area and volume as
    // a genus-1 shell (the two branch arms one seamless face), and point-in-
    // face then read 605 of 777 arm points outside it.
    let mut used = BTreeSet::new();
    for triangle in &triangles {
        used.extend(triangle.iter().copied());
    }
    let mesh_chi = used.len() as i64 - incidence.len() as i64 + triangles.len() as i64;
    let output_genus = brep_kernel::solid_euler(&output.solid).genus;
    if mesh_chi % 2 == 0 && output_genus.is_some_and(|genus| genus != (2 - mesh_chi) / 2) {
        return Err(fail(format!(
            "hybrid BREP: the rebuilt shell has genus {} but the source mesh has genus {}",
            output_genus.unwrap_or(-1),
            (2 - mesh_chi) / 2
        )));
    }
    Ok(output)
}

/// Convert the assembled arena and run the whole-body checks.
fn finish_shell(arena: TopologyArena, stats: HybridBrepStats) -> Result<HybridBrepOutput, String> {
    let mut solid = arena.to_brep()?;
    stage_trace("arena converted");
    let vertex_count = solid.vertices.len() as i64;
    // A degenerate pole edge is not a 1-cell of the shell.
    let edge_count = solid.edges.iter().filter(|edge| !edge.degenerate).count() as i64;
    let face_count = solid
        .shells
        .iter()
        .map(|shell| shell.faces.len())
        .sum::<usize>() as i64;
    let extra_loops = solid
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .map(|face| face.loops.len().saturating_sub(1) as i64)
        .sum::<i64>();
    let numerator = 2 - (vertex_count - edge_count + face_count - extra_loops);
    if numerator < 0 || numerator % 2 != 0 {
        return Err(format!(
            "hybrid BREP: non-manifold Euler accounting (2-(V-E+F-H)={numerator})"
        ));
    }
    solid.genus = numerator / 2;
    let issues = solid.validate();
    stage_trace("validated");
    if !issues.is_empty() {
        return Err(format!("hybrid BREP: topology validation failed: {issues:?}"));
    }
    let volume = solid_signed_volume(&solid)?;
    stage_trace("volume computed");
    if !volume.is_finite() || volume <= 0.0 {
        return Err(format!(
            "hybrid BREP: reconstructed shell is inverted or empty (signed volume {volume:.6e})"
        ));
    }
    Ok(HybridBrepOutput { solid, stats })
}

fn validate_cone_span(
    original: u32,
    low: f64,
    high: f64,
    half_angle: f64,
    tolerance: f64,
) -> Result<(), HybridBuildFailure> {
    if !low.is_finite() || high - low <= tolerance || low * half_angle.tan() <= tolerance {
        return Err(HybridBuildFailure::Region {
            original,
            message: "pointed or zero-span cone requires unsupported pole topology".into(),
            cause: DemotionCause::ConeSpan,
        });
    }
    Ok(())
}

fn mesh_diagonal(vertices: &[Vec3]) -> f64 {
    let mut low = vertices[0];
    let mut high = vertices[0];
    for &point in &vertices[1..] {
        low.x = low.x.min(point.x);
        low.y = low.y.min(point.y);
        low.z = low.z.min(point.z);
        high.x = high.x.max(point.x);
        high.y = high.y.max(point.y);
        high.z = high.z.max(point.z);
    }
    high.sub(low).length()
}

type EdgeKey = (usize, usize);

fn edge_key(a: usize, b: usize) -> EdgeKey {
    if a < b {
        (a, b)
    } else {
        (b, a)
    }
}

fn edge_incidence(triangles: &[[usize; 3]]) -> Result<BTreeMap<EdgeKey, Vec<usize>>, String> {
    let mut incidence = BTreeMap::<EdgeKey, Vec<usize>>::new();
    for (triangle_id, triangle) in triangles.iter().enumerate() {
        if triangle[0] == triangle[1] || triangle[1] == triangle[2] || triangle[2] == triangle[0] {
            return Err("hybrid BREP: repeated vertex in triangle".into());
        }
        for corner in 0..3 {
            incidence
                .entry(edge_key(triangle[corner], triangle[(corner + 1) % 3]))
                .or_default()
                .push(triangle_id);
        }
    }
    Ok(incidence)
}

fn validate_closed_incidence(
    triangles: &[[usize; 3]],
    incidence: &BTreeMap<EdgeKey, Vec<usize>>,
) -> Result<(), String> {
    for (&(a, b), uses) in incidence {
        if uses.len() != 2 {
            return Err("hybrid BREP: input is not a closed two-manifold".into());
        }
        let forward = |t: usize| {
            (0..3).any(|i| triangles[t][i] == a && triangles[t][(i + 1) % 3] == b)
        };
        if forward(uses[0]) == forward(uses[1]) {
            return Err("hybrid BREP: adjacent triangles have inconsistent winding".into());
        }
    }
    Ok(())
}

fn carrier_cylinder(region: &MeshRegion, pair_section_bar: bool) -> Result<Option<CylinderCarrier>, String> {
    let RegionCarrier::Cylinder {
        axis_point,
        axis_dir,
        radius,
        sense,
    } = region.carrier
    else {
        return Ok(None);
    };
    if !radius.is_finite() || radius <= 0.0 || !matches!(sense, -1 | 1) {
        return Ok(None);
    }
    Ok(Some(CylinderCarrier {
        origin: axis_point,
        axis: axis_dir.normalized()?,
        radius,
        sense,
        deviation: region.max_deviation.max(0.0),
        pair_section_bar,
    }))
}

fn carrier_cone(region: &MeshRegion) -> Result<Option<ConeCarrier>, String> {
    let RegionCarrier::Cone {
        apex,
        axis_dir,
        half_angle_rad,
        sense,
    } = region.carrier
    else {
        return Ok(None);
    };
    if !half_angle_rad.is_finite()
        || !(1.0e-6..std::f64::consts::FRAC_PI_2 - 1.0e-6).contains(&half_angle_rad)
        || !matches!(sense, -1 | 1)
    {
        return Ok(None);
    }
    Ok(Some(ConeCarrier {
        apex,
        axis: axis_dir.normalized()?,
        half_angle: half_angle_rad,
        sense,
        deviation: region.max_deviation.max(0.0),
    }))
}

fn carrier_sphere(region: &MeshRegion) -> Result<Option<SphereCarrier>, String> {
    let RegionCarrier::Sphere {
        center,
        radius,
        sense,
    } = region.carrier
    else {
        return Ok(None);
    };
    if !radius.is_finite() || radius <= 0.0 || !matches!(sense, -1 | 1) {
        return Ok(None);
    }
    Ok(Some(SphereCarrier {
        center,
        radius,
        sense,
        deviation: region.max_deviation.max(0.0),
    }))
}

fn sphere_radial_error(sphere: SphereCarrier, point: Vec3) -> f64 {
    (point.sub(sphere.center).length() - sphere.radius).abs()
}

/// A sphere/plane edge is a circle whenever both ends lie in the plane and
/// on the sphere; whether that circle is a ring of the chosen polar axis is
/// settled when the sphere surface is framed.
fn sphere_plane_edge_supported(
    sphere: SphereCarrier,
    plane: PlaneInfo,
    a: Vec3,
    b: Vec3,
    tolerance: f64,
) -> bool {
    let Ok(normal) = plane.normal.normalized() else {
        return false;
    };
    let bar = accepted_radial_bar(tolerance, sphere.deviation);
    a.sub(plane.origin).dot(normal).abs() <= tolerance
        && b.sub(plane.origin).dot(normal).abs() <= tolerance
        && sphere_radial_error(sphere, a) <= bar
        && sphere_radial_error(sphere, b) <= bar
}

/// A sphere meets a coaxial revolve (its center on the axis) along a ring:
/// both ends at one station on the axis, on the sphere and on the revolve.
fn sphere_revolve_edge_supported(
    sphere: SphereCarrier,
    axis_origin: Vec3,
    axis: Vec3,
    on_revolve: impl Fn(Vec3) -> bool,
    a: Vec3,
    b: Vec3,
    tolerance: f64,
) -> bool {
    let offset = sphere.center.sub(axis_origin);
    if offset.sub(axis.scale(offset.dot(axis))).length() > tolerance * 4.0 {
        return false;
    }
    let bar = accepted_radial_bar(tolerance * 4.0, sphere.deviation);
    (a.sub(axis_origin).dot(axis) - b.sub(axis_origin).dot(axis)).abs() <= tolerance * 4.0
        && sphere_radial_error(sphere, a) <= bar
        && sphere_radial_error(sphere, b) <= bar
        && on_revolve(a)
        && on_revolve(b)
}

fn sphere_cylinder_edge_supported(
    sphere: SphereCarrier,
    cylinder: CylinderCarrier,
    a: Vec3,
    b: Vec3,
    tolerance: f64,
) -> bool {
    sphere_revolve_edge_supported(
        sphere,
        cylinder.origin,
        cylinder.axis,
        |point| {
            (radial(cylinder, point).length() - cylinder.radius).abs()
                <= accepted_radial_bar(tolerance * 4.0, cylinder.deviation)
        },
        a,
        b,
        tolerance,
    )
}

fn sphere_cone_edge_supported(
    sphere: SphereCarrier,
    cone: ConeCarrier,
    a: Vec3,
    b: Vec3,
    tolerance: f64,
) -> bool {
    sphere_revolve_edge_supported(
        sphere,
        cone.apex,
        cone.axis,
        |point| point_on_cone(cone, point, accepted_radial_bar(tolerance * 4.0, cone.deviation)),
        a,
        b,
        tolerance,
    )
}

fn carrier_torus(region: &MeshRegion) -> Result<Option<TorusCarrier>, String> {
    let RegionCarrier::Torus {
        center,
        axis_dir,
        major_radius,
        minor_radius,
        sense,
    } = region.carrier
    else {
        return Ok(None);
    };
    let Ok(axis) = axis_dir.normalized() else {
        return Ok(None);
    };
    if !major_radius.is_finite()
        || !minor_radius.is_finite()
        || minor_radius <= 0.0
        || major_radius <= minor_radius
        || !matches!(sense, -1 | 1)
    {
        return Ok(None);
    }
    Ok(Some(TorusCarrier {
        center,
        axis,
        major_radius,
        minor_radius,
        sense,
        deviation: region.max_deviation.max(0.0),
    }))
}

fn torus_radial_error(torus: TorusCarrier, point: Vec3) -> f64 {
    let d = point.sub(torus.center);
    let z = d.dot(torus.axis);
    let rho = d.sub(torus.axis.scale(z)).length();
    (((rho - torus.major_radius).powi(2) + z * z).sqrt() - torus.minor_radius).abs()
}

/// A torus meets a plane square to its axis along a latitude ring: both
/// ends in the plane and on the torus.
fn torus_plane_edge_supported(
    torus: TorusCarrier,
    plane: PlaneInfo,
    a: Vec3,
    b: Vec3,
    tolerance: f64,
) -> bool {
    let Ok(normal) = plane.normal.normalized() else {
        return false;
    };
    let bar = accepted_radial_bar(tolerance, torus.deviation);
    normal.cross(torus.axis).length() * torus.major_radius <= tolerance * 4.0
        && a.sub(plane.origin).dot(normal).abs() <= tolerance
        && b.sub(plane.origin).dot(normal).abs() <= tolerance
        && torus_radial_error(torus, a) <= bar
        && torus_radial_error(torus, b) <= bar
}

/// A torus meets a coaxial revolve (the same axis line) along a latitude
/// ring: both ends at one station, on the torus and on the revolve.
fn torus_revolve_edge_supported(
    torus: TorusCarrier,
    axis_origin: Vec3,
    axis: Vec3,
    on_revolve: impl Fn(Vec3) -> bool,
    a: Vec3,
    b: Vec3,
    tolerance: f64,
) -> bool {
    let offset = torus.center.sub(axis_origin);
    if offset.sub(axis.scale(offset.dot(axis))).length() > tolerance * 4.0
        || axis.cross(torus.axis).length() * torus.major_radius > tolerance * 4.0
    {
        return false;
    }
    let bar = accepted_radial_bar(tolerance * 4.0, torus.deviation);
    (a.sub(axis_origin).dot(axis) - b.sub(axis_origin).dot(axis)).abs() <= tolerance * 4.0
        && torus_radial_error(torus, a) <= bar
        && torus_radial_error(torus, b) <= bar
        && on_revolve(a)
        && on_revolve(b)
}

fn edge_is_cylinder_ruling(cylinder: CylinderCarrier, a: Vec3, b: Vec3, tolerance: f64) -> bool {
    let ra = radial(cylinder, a);
    let rb = radial(cylinder, b);
    let Ok(ua) = ra.normalized() else {
        return false;
    };
    let Ok(ub) = rb.normalized() else {
        return false;
    };
    let chord = b.sub(a);
    let Ok(direction) = chord.normalized() else {
        return false;
    };
    (ra.length() - cylinder.radius).abs() <= tolerance
        && (rb.length() - cylinder.radius).abs() <= tolerance
        && ua.cross(ub).length() <= 0.01
        && direction.cross(cylinder.axis).length() <= 0.01
}

/// How far a crossing's axes may miss each other and an edge point may sit
/// off a section plane: the bare exact-topology bar, or, under
/// [`HybridSwitches::pair_section_bar`], the radial bar the walls' deviation
/// already earns their radius.
fn cylinder_pair_section_bar(first: CylinderCarrier, second: CylinderCarrier, tolerance: f64) -> f64 {
    if first.pair_section_bar || second.pair_section_bar {
        accepted_radial_bar(tolerance * 4.0, first.deviation.max(second.deviation))
    } else {
        tolerance * 4.0
    }
}

/// Where two cylinders of one radius whose axes cross meet: the two planes
/// through the crossing point normal to the sum and to the difference of the
/// axes (each cuts both walls in the same ellipse), or `None` when the pair
/// is not such a crossing within the exact-topology bar.
fn cylinder_pair_section(
    first: CylinderCarrier,
    second: CylinderCarrier,
    tolerance: f64,
) -> Option<(Vec3, [Vec3; 2])> {
    let bar = accepted_radial_bar(tolerance * 4.0, first.deviation.max(second.deviation));
    if (first.radius - second.radius).abs() > bar {
        return None;
    }
    let (a1, a2) = (first.axis.normalized().ok()?, second.axis.normalized().ok()?);
    let across = a1.cross(a2);
    if across.length() <= 0.1 {
        return None;
    }
    // Closest points of the two axis lines.
    let w0 = first.origin.sub(second.origin);
    let (b, d, e) = (a1.dot(a2), a1.dot(w0), a2.dot(w0));
    let denominator = 1.0 - b * b;
    let s = (b * e - d) / denominator;
    let t = (e - b * d) / denominator;
    let p1 = first.origin.add(a1.scale(s));
    let p2 = second.origin.add(a2.scale(t));
    if p1.sub(p2).length() > cylinder_pair_section_bar(first, second, tolerance) {
        return None;
    }
    let normals = [a1.add(a2).normalized().ok()?, a1.sub(a2).normalized().ok()?];
    Some((p1.add(p2).scale(0.5), normals))
}

/// A mesh edge that cuts the corner at a tangent point of two equal-radius
/// crossing cylinders: both ends on both walls, one end on each section
/// plane, and the tangent point `center ± r·(a1 × a2)` closer to the edge than
/// the edge is long. `split_crossing_cylinder_chains` moves the end nearest
/// the tangent point onto it, so the edge is not a reason to demote.
fn cylinder_pair_corner_cut(
    first: CylinderCarrier,
    second: CylinderCarrier,
    a: Vec3,
    b: Vec3,
    tolerance: f64,
) -> bool {
    let Some((center, normals)) = cylinder_pair_section(first, second, tolerance) else {
        return false;
    };
    let bar = |cylinder: CylinderCarrier| accepted_radial_bar(tolerance * 4.0, cylinder.deviation);
    if [a, b].iter().any(|&point| {
        (radial(first, point).length() - first.radius).abs() > bar(first)
            || (radial(second, point).length() - second.radius).abs() > bar(second)
    }) {
        return false;
    }
    let plane_bar = cylinder_pair_section_bar(first, second, tolerance);
    let on = |point: Vec3, k: usize| point.sub(center).dot(normals[k]).abs() <= plane_bar;
    let Ok(minor) = first.axis.cross(second.axis).normalized() else {
        return false;
    };
    let radius = 0.5 * (first.radius + second.radius);
    let length = b.sub(a).length();
    let tangents = [center.add(minor.scale(radius)), center.sub(minor.scale(radius))];
    if !((on(a, 0) && on(b, 1)) || (on(a, 1) && on(b, 0))) {
        // Under the pair-section bar: near a tangent point the two walls are
        // tangent to each other, so a point within the radial bar `e` of both
        // sits up to about sqrt(2 r e) off both section planes. One end on a
        // plane and the other that close to a tangent point is still the
        // corner cut `split_crossing_cylinder_chains` moves onto it.
        if !(first.pair_section_bar || second.pair_section_bar) {
            return false;
        }
        let reach = (2.0 * radius * plane_bar).sqrt();
        let near = |point: Vec3| tangents.iter().any(|&tangent| tangent.sub(point).length() <= reach.min(length));
        let on_any = |point: Vec3| on(point, 0) || on(point, 1);
        return (on_any(a) && near(b)) || (on_any(b) && near(a));
    }
    tangents
        .iter()
        .any(|&tangent| tangent.sub(a).length().min(tangent.sub(b).length()) < length)
}

/// The section plane normal a cylinder/cylinder edge lies in, when both
/// ends are on both walls and on one plane of [`cylinder_pair_section`].
fn cylinder_pair_section_normal(
    first: CylinderCarrier,
    second: CylinderCarrier,
    points: &[Vec3],
    tolerance: f64,
) -> Option<(Vec3, Vec3)> {
    let (center, normals) = cylinder_pair_section(first, second, tolerance)?;
    let bar = |cylinder: CylinderCarrier| accepted_radial_bar(tolerance * 4.0, cylinder.deviation);
    if points.iter().any(|&point| {
        (radial(first, point).length() - first.radius).abs() > bar(first)
            || (radial(second, point).length() - second.radius).abs() > bar(second)
    }) {
        return None;
    }
    let plane_bar = cylinder_pair_section_bar(first, second, tolerance);
    normals
        .into_iter()
        .find(|normal| {
            points
                .iter()
                .all(|point| point.sub(center).dot(*normal).abs() <= plane_bar)
        })
        .map(|normal| (center, normal))
}

/// [`HybridSwitches::cap_ring`]: both ends on a plane square to the
/// cylinder's axis within 0.01 and within the radial bar of the wall about
/// the axis squared to that plane (through the same origin), as
/// `snap_cylinder_axes` leaves it.
fn cap_ring_supported(cylinder: CylinderCarrier, plane: PlaneInfo, a: Vec3, b: Vec3, tolerance: f64) -> bool {
    let Ok(normal) = plane.normal.normalized() else {
        return false;
    };
    if normal.cross(cylinder.axis).length() > 0.01 {
        return false;
    }
    let snapped = CylinderCarrier {
        axis: if normal.dot(cylinder.axis) >= 0.0 { normal } else { normal.scale(-1.0) },
        ..cylinder
    };
    [a, b].iter().all(|&point| {
        point.sub(plane.origin).dot(normal).abs() <= tolerance
            && (radial(snapped, point).length() - snapped.radius).abs()
                <= accepted_radial_bar(tolerance * 4.0, snapped.deviation)
    })
}

fn cylinder_plane_edge_supported(
    cylinder: CylinderCarrier,
    plane: PlaneInfo,
    a: Vec3,
    b: Vec3,
    tolerance: f64,
) -> bool {
    let Ok(normal) = plane.normal.normalized() else {
        return false;
    };
    if a.sub(plane.origin).dot(normal).abs() > tolerance
        || b.sub(plane.origin).dot(normal).abs() > tolerance
    {
        return false;
    }
    let ra = radial(cylinder, a);
    let rb = radial(cylinder, b);
    let looks_like_ruling = (station(cylinder, a) - station(cylinder, b)).abs() > tolerance
        && ra
            .normalized()
            .ok()
            .zip(rb.normalized().ok())
            .is_some_and(|(ua, ub)| ua.cross(ub).length() <= 0.05);
    if looks_like_ruling {
        // The exact axial line is contained by the plane only when its
        // direction is tangent to that plane.
        return normal.dot(cylinder.axis).abs() <= 0.01;
    }
    let on_ring = (station(cylinder, a) - station(cylinder, b)).abs() <= tolerance * 4.0;
    // A constant-station circle lies in the neighboring plane only when that
    // plane is perpendicular to the cylinder axis.
    on_ring && normal.cross(cylinder.axis).length() <= 0.01
}

fn cone_station(cone: ConeCarrier, point: Vec3) -> f64 {
    point.sub(cone.apex).dot(cone.axis)
}

fn cone_radial(cone: ConeCarrier, point: Vec3) -> Vec3 {
    let delta = point.sub(cone.apex);
    delta.sub(cone.axis.scale(delta.dot(cone.axis)))
}

fn point_on_cone(cone: ConeCarrier, point: Vec3, tolerance: f64) -> bool {
    let station = cone_station(cone, point);
    station >= -tolerance
        && (cone_radial(cone, point).length() - station * cone.half_angle.tan()).abs() <= tolerance
}

fn edge_is_cone_ruling(cone: ConeCarrier, a: Vec3, b: Vec3, tolerance: f64) -> bool {
    if !point_on_cone(cone, a, tolerance) || !point_on_cone(cone, b, tolerance) {
        return false;
    }
    let ra = cone_radial(cone, a);
    let rb = cone_radial(cone, b);
    let (Ok(ua), Ok(ub), Ok(direction)) = (ra.normalized(), rb.normalized(), b.sub(a).normalized())
    else {
        return false;
    };
    let expected = cone
        .axis
        .add(ua.scale(cone.half_angle.tan()))
        .normalized()
        .unwrap_or(cone.axis);
    ua.cross(ub).length() <= 0.01
        && direction
            .cross(expected)
            .length()
            .min(direction.cross(expected.scale(-1.0)).length())
            <= 0.01
}

fn cone_plane_edge_supported(
    cone: ConeCarrier,
    plane: PlaneInfo,
    a: Vec3,
    b: Vec3,
    tolerance: f64,
) -> bool {
    let Ok(normal) = plane.normal.normalized() else {
        return false;
    };
    let da = a.sub(plane.origin).dot(normal).abs();
    let db = b.sub(plane.origin).dot(normal).abs();
    if da > tolerance || db > tolerance {
        return false;
    }
    let ra = cone_radial(cone, a);
    let rb = cone_radial(cone, b);
    let looks_like_ruling = (cone_station(cone, a) - cone_station(cone, b)).abs() > tolerance
        && ra
            .normalized()
            .ok()
            .zip(rb.normalized().ok())
            .is_some_and(|(ua, ub)| ua.cross(ub).length() <= 0.05);
    if looks_like_ruling {
        let apex_in_plane = cone.apex.sub(plane.origin).dot(normal).abs() <= tolerance * 4.0;
        if apex_in_plane && normal.dot(b.sub(a)).abs() <= tolerance {
            return true;
        }
    }
    let on_ring = (cone_station(cone, a) - cone_station(cone, b)).abs() <= tolerance * 4.0;
    if on_ring && normal.cross(cone.axis).length() <= 0.01 {
        return true;
    }
    // A finite oblique trim is represented later as the exact projective map
    // of only its reference-circle arc. This also covers local hyperbolic and
    // parabolic branches without the mixed weights of a full-circle map.
    true
}

fn cylinder_cone_edge_supported(
    cylinder: CylinderCarrier,
    cone: ConeCarrier,
    a: Vec3,
    b: Vec3,
    tolerance: f64,
) -> bool {
    if cylinder.axis.cross(cone.axis).length() > 0.01 {
        return false;
    }
    let cylinder_station_spread = (station(cylinder, a) - station(cylinder, b)).abs();
    let cone_station_spread = (cone_station(cone, a) - cone_station(cone, b)).abs();
    cylinder_station_spread <= tolerance * 4.0
        && cone_station_spread <= tolerance * 4.0
        && (radial(cylinder, a).length() - cylinder.radius).abs() <= tolerance * 4.0
        && (radial(cylinder, b).length() - cylinder.radius).abs() <= tolerance * 4.0
        && point_on_cone(cone, a, tolerance * 4.0)
        && point_on_cone(cone, b, tolerance * 4.0)
}

/// Two coaxial cones can share an exact perpendicular circle, including
/// opposing tapers. Reject skew axes and non-ring intersections.
fn cone_cone_edge_supported(
    first: ConeCarrier,
    second: ConeCarrier,
    a: Vec3,
    b: Vec3,
    tolerance: f64,
) -> bool {
    first.axis.cross(second.axis).length() <= 1.0e-8
        && cone_radial(first, second.apex).length() <= tolerance * 4.0
        && [first, second].iter().all(|&cone| {
            (cone_station(cone, a) - cone_station(cone, b)).abs() <= tolerance * 4.0
                && point_on_cone(cone, a, tolerance * 4.0)
                && point_on_cone(cone, b, tolerance * 4.0)
        })
}

/// Diagnostic stage trace, enabled by `BREP_DEBUG_HYBRID`.
fn stage_trace(stage: &str) {
    if std::env::var("BREP_DEBUG_HYBRID").is_ok() {
        eprintln!("[hybrid] stage: {stage}");
    }
}

/// Diagnostic trace of one demotion decision, enabled by `BREP_DEBUG_HYBRID`.
fn demote_trace(candidate: u32, neighbor: u32, why: &str, a: Vec3, b: Vec3) {
    if std::env::var("BREP_DEBUG_HYBRID").is_ok() {
        eprintln!(
            "[hybrid] demote region {candidate} (neighbour {neighbor}): {why}; edge ({:.4},{:.4},{:.4})-({:.4},{:.4},{:.4})",
            a.x, a.y, a.z, b.x, b.y, b.z
        );
    }
}

fn facet_plane(triangle: [usize; 3], vertices: &[Vec3]) -> Result<PlaneInfo, String> {
    let a = vertices[triangle[0]];
    let b = vertices[triangle[1]];
    let c = vertices[triangle[2]];
    let normal = b.sub(a).cross(c.sub(a)).normalized().map_err(|_| {
        "hybrid BREP: cannot create a planar face from a degenerate source triangle".to_owned()
    })?;
    Ok(PlaneInfo { origin: a, normal })
}

fn facets_are_coplanar(
    first: [usize; 3],
    second: [usize; 3],
    vertices: &[Vec3],
    tolerance: f64,
) -> Result<bool, String> {
    let first_plane = facet_plane(first, vertices)?;
    let second_plane = facet_plane(second, vertices)?;
    // Do not turn a shallow crease into a plane merely because a distance
    // tolerance happens to cover its short triangles. The distance checks
    // below cover f32 STL coordinate noise; this angular check remains
    // deliberately close to exact coplanarity.
    if first_plane.normal.dot(second_plane.normal) <= 0.0
        || first_plane.normal.cross(second_plane.normal).length() > 1.0e-8
    {
        return Ok(false);
    }
    Ok(first.iter().chain(second.iter()).all(|&vertex| {
        let point = vertices[vertex];
        point.sub(first_plane.origin).dot(first_plane.normal).abs() <= tolerance
            && point
                .sub(second_plane.origin)
                .dot(second_plane.normal)
                .abs()
                <= tolerance
    }))
}

/// Replace unsupported regions with safely merged coplanar patches and
/// recursively demote revolved regions that cannot share exact edges with them.
fn local_regions(
    regions: &[MeshRegion],
    triangle_regions: &[u32],
    triangles: &[[usize; 3]],
    vertices: &[Vec3],
    incidence: &BTreeMap<EdgeKey, Vec<usize>>,
    tolerance: f64,
    forced_demotions: &BTreeMap<u32, (DemotionCause, String)>,
    switches: HybridSwitches,
    parent: Option<&[usize]>,
) -> Result<LocalRegions, String> {
    let mut original_cylinders = HashMap::<u32, CylinderCarrier>::new();
    let mut original_cones = HashMap::<u32, ConeCarrier>::new();
    let mut original_spheres = HashMap::<u32, SphereCarrier>::new();
    let mut original_tori = HashMap::<u32, TorusCarrier>::new();
    let mut original_planes = HashMap::<u32, PlaneInfo>::new();
    let mut unresolved_regions = BTreeSet::new();
    let mut retained = BTreeSet::<u32>::new();
    for region in regions {
        if matches!(region.carrier, RegionCarrier::Freeform) {
            unresolved_regions.insert(region.id);
        }
        if let RegionCarrier::Plane { origin, normal } = region.carrier {
            original_planes.insert(region.id, PlaneInfo { origin, normal });
        }
        if let Some(cylinder) = carrier_cylinder(region, switches.pair_section_bar)? {
            original_cylinders.insert(region.id, cylinder);
            if !forced_demotions.contains_key(&region.id) {
                retained.insert(region.id);
            }
        }
        if let Some(cone) = carrier_cone(region)? {
            original_cones.insert(region.id, cone);
            if !forced_demotions.contains_key(&region.id) {
                retained.insert(region.id);
            }
        }
        if let Some(sphere) = carrier_sphere(region)? {
            original_spheres.insert(region.id, sphere);
            if !forced_demotions.contains_key(&region.id) {
                retained.insert(region.id);
            }
        }
        if let Some(torus) = carrier_torus(region)? {
            original_tori.insert(region.id, torus);
            if !forced_demotions.contains_key(&region.id) {
                retained.insert(region.id);
            }
        }
    }
    #[derive(Clone, Copy)]
    enum Revolve {
        Cylinder(CylinderCarrier),
        Cone(ConeCarrier),
        Sphere(SphereCarrier),
        Torus(TorusCarrier),
    }
    let revolve_of = |id: u32| -> Option<Revolve> {
        original_cylinders
            .get(&id)
            .map(|&cylinder| Revolve::Cylinder(cylinder))
            .or_else(|| original_cones.get(&id).map(|&cone| Revolve::Cone(cone)))
            .or_else(|| original_spheres.get(&id).map(|&sphere| Revolve::Sphere(sphere)))
            .or_else(|| original_tori.get(&id).map(|&torus| Revolve::Torus(torus)))
    };
    // A sphere ring stays exact against a coaxial revolve even when that
    // revolve was itself demoted for one of its other trims.
    let sphere_ring_with = |sphere: SphereCarrier, other: Revolve, a: Vec3, b: Vec3| -> bool {
        match other {
            Revolve::Cylinder(cylinder) => {
                sphere_cylinder_edge_supported(sphere, cylinder, a, b, tolerance)
            }
            Revolve::Cone(cone) => sphere_cone_edge_supported(sphere, cone, a, b, tolerance),
            Revolve::Sphere(_) | Revolve::Torus(_) => false,
        }
    };
    // A torus ring stays exact against a coaxial cylinder, cone or torus in
    // the same way.
    let torus_ring_with = |torus: TorusCarrier, other: Revolve, a: Vec3, b: Vec3| -> bool {
        match other {
            Revolve::Cylinder(cylinder) => torus_revolve_edge_supported(
                torus,
                cylinder.origin,
                cylinder.axis,
                |point| {
                    (radial(cylinder, point).length() - cylinder.radius).abs()
                        <= accepted_radial_bar(tolerance * 4.0, cylinder.deviation)
                },
                a,
                b,
                tolerance,
            ),
            Revolve::Cone(cone) => torus_revolve_edge_supported(
                torus,
                cone.apex,
                cone.axis,
                |point| {
                    point_on_cone(cone, point, accepted_radial_bar(tolerance * 4.0, cone.deviation))
                },
                a,
                b,
                tolerance,
            ),
            Revolve::Torus(other) => torus_revolve_edge_supported(
                torus,
                other.center,
                other.axis,
                |point| {
                    torus_radial_error(other, point)
                        <= accepted_radial_bar(tolerance * 4.0, other.deviation)
                },
                a,
                b,
                tolerance,
            ),
            Revolve::Sphere(_) => false,
        }
    };

    // Lane V row 1: the first cause that removed each region from
    // `retained`. Within one round a direct cause replaces a recursive one,
    // so a recursive demotion is one no direct check would have made.
    let mut causes = BTreeMap::<u32, (DemotionCause, Option<u32>, usize)>::new();
    // [`HybridSwitches::cap_ring`]: region pairs whose shared edges close
    // into loops (every vertex on two of them), the cap loops
    // `snap_cylinder_axes` squares a cylinder's axis to.
    let mut closed_pairs = BTreeSet::<(u32, u32)>::new();
    if switches.cap_ring {
        let mut degree = BTreeMap::<(u32, u32), HashMap<usize, usize>>::new();
        for (&(a, b), incident) in incidence {
            let (first, second) = (triangle_regions[incident[0]], triangle_regions[incident[1]]);
            if first != second {
                let entry = degree.entry((first.min(second), first.max(second))).or_default();
                *entry.entry(a).or_default() += 1;
                *entry.entry(b).or_default() += 1;
            }
        }
        closed_pairs.extend(
            degree
                .into_iter()
                .filter(|(_, counts)| counts.values().all(|&count| count == 2))
                .map(|(pair, _)| pair),
        );
    }
    let mut round = 0_usize;
    loop {
        round += 1;
        let mut demote = BTreeSet::new();
        let mut round_causes = BTreeMap::<u32, (DemotionCause, Option<u32>)>::new();
        let mut note = |region: u32, cause: DemotionCause, neighbor: u32| {
            let entry = round_causes.entry(region).or_insert((cause, Some(neighbor)));
            if entry.0 == DemotionCause::DemotedNeighbourNotRuling
                && cause != DemotionCause::DemotedNeighbourNotRuling
            {
                *entry = (cause, Some(neighbor));
            }
        };
        for (&(a, b), incident) in incidence {
            let first = triangle_regions[incident[0]];
            let second = triangle_regions[incident[1]];
            if first == second {
                continue;
            }
            for (candidate, neighbor) in [(first, second), (second, first)] {
                if !retained.contains(&candidate) {
                    continue;
                }
                let Some(own) = revolve_of(candidate) else {
                    continue;
                };
                if retained.contains(&neighbor) {
                    let (supported, why, cause) = match (own, revolve_of(neighbor)) {
                        (Revolve::Cylinder(cylinder), Some(Revolve::Cone(cone)))
                        | (Revolve::Cone(cone), Some(Revolve::Cylinder(cylinder))) => (
                            cylinder_cone_edge_supported(
                                cylinder,
                                cone,
                                vertices[a],
                                vertices[b],
                                tolerance,
                            ),
                            "cylinder/cone edge unsupported",
                            DemotionCause::CylinderConeEdge,
                        ),
                        // Two cylinders meet exactly along a shared axial
                        // ruling: a tangent chain of arcs on parallel axes
                        // (a rounded outline, a fillet running out into a
                        // boss). Any other cylinder/cylinder curve is not
                        // yet proven exact.
                        // Two cylinders of one radius whose axes cross meet
                        // in planar ellipses (crossing bores).
                        (Revolve::Cylinder(first), Some(Revolve::Cylinder(second))) => (
                            (edge_is_cylinder_ruling(first, vertices[a], vertices[b], tolerance)
                                && edge_is_cylinder_ruling(
                                    second,
                                    vertices[a],
                                    vertices[b],
                                    tolerance,
                                ))
                                || cylinder_pair_section_normal(
                                    first,
                                    second,
                                    &[vertices[a], vertices[b]],
                                    tolerance,
                                )
                                .is_some()
                                || cylinder_pair_corner_cut(
                                    first,
                                    second,
                                    vertices[a],
                                    vertices[b],
                                    tolerance,
                                ),
                            "cylinder/cylinder edge is neither a shared ruling nor an equal-radius crossing",
                            DemotionCause::CylinderCylinderEdge,
                        ),
                        (Revolve::Sphere(sphere), Some(other))
                        | (other, Some(Revolve::Sphere(sphere))) => (
                            sphere_ring_with(sphere, other, vertices[a], vertices[b]),
                            "sphere edge is not a coaxial ring",
                            DemotionCause::SphereRingEdge,
                        ),
                        (Revolve::Torus(torus), Some(other))
                        | (other, Some(Revolve::Torus(torus))) => (
                            torus_ring_with(torus, other, vertices[a], vertices[b]),
                            "torus edge is not a coaxial ring",
                            DemotionCause::TorusRingEdge,
                        ),
                        (Revolve::Cone(first), Some(Revolve::Cone(second))) => (
                            cone_cone_edge_supported(first, second, vertices[a], vertices[b], tolerance),
                            "cone/cone edge is not a coaxial ring",
                            DemotionCause::ConeConeEdge,
                        ),
                        _ => (false, "same-kind revolve adjacency", DemotionCause::SameKindRevolveEdge),
                    };
                    if !supported {
                        demote_trace(candidate, neighbor, why, vertices[a], vertices[b]);
                        demote.insert(candidate);
                        demote.insert(neighbor);
                        note(candidate, cause, neighbor);
                        note(neighbor, cause, candidate);
                    }
                    continue;
                }
                // A revolve may have been locally demoted because one of its
                // other trims is unsupported. Its shared ring with a coaxial
                // neighbor remains exact, so that neighbor need not be
                // demoted with it.
                match (own, revolve_of(neighbor)) {
                    (Revolve::Cylinder(cylinder), Some(Revolve::Cone(cone))) => {
                        if cylinder_cone_edge_supported(
                            cylinder,
                            cone,
                            vertices[a],
                            vertices[b],
                            tolerance,
                        ) {
                            continue;
                        }
                    }
                    (Revolve::Sphere(sphere), Some(other)) => {
                        if sphere_ring_with(sphere, other, vertices[a], vertices[b]) {
                            continue;
                        }
                    }
                    (other, Some(Revolve::Sphere(sphere))) => {
                        if sphere_ring_with(sphere, other, vertices[a], vertices[b]) {
                            continue;
                        }
                    }
                    (Revolve::Torus(torus), Some(other)) | (other, Some(Revolve::Torus(torus))) => {
                        if torus_ring_with(torus, other, vertices[a], vertices[b]) {
                            continue;
                        }
                    }
                    _ => {}
                }
                // An unresolved compound is later split into coplanar patches.
                // Its facet at this interface may already be a valid planar
                // trim (for example a small bore cap below the segmenter's
                // planar area cutoff). Judge that actual support before
                // discarding the entire curved face as a non-planar adjacency.
                let neighbor_plane = if let Some(&plane) = original_planes.get(&neighbor) {
                    Some((plane, DemotionCause::PlaneNeighbourEdge))
                } else if unresolved_regions.contains(&neighbor) {
                    let triangle = incident
                        .iter()
                        .copied()
                        .find(|&t| triangle_regions[t] == neighbor)
                        .ok_or_else(|| {
                            "hybrid BREP: missing unresolved interface triangle".to_owned()
                        })?;
                    Some((facet_plane(triangles[triangle], vertices)?, DemotionCause::FacetNeighbourEdge))
                } else {
                    None
                };
                if let Some((plane, cause)) = neighbor_plane {
                    let supported = match own {
                        Revolve::Cylinder(cylinder) => {
                            cylinder_plane_edge_supported(cylinder, plane, vertices[a], vertices[b], tolerance)
                                || (cause == DemotionCause::PlaneNeighbourEdge
                                    && closed_pairs.contains(&(candidate.min(neighbor), candidate.max(neighbor)))
                                    && cap_ring_supported(cylinder, plane, vertices[a], vertices[b], tolerance))
                        }
                        Revolve::Cone(cone) => cone_plane_edge_supported(
                            cone,
                            plane,
                            vertices[a],
                            vertices[b],
                            tolerance,
                        ),
                        Revolve::Sphere(sphere) => sphere_plane_edge_supported(
                            sphere,
                            plane,
                            vertices[a],
                            vertices[b],
                            tolerance,
                        ),
                        Revolve::Torus(torus) => torus_plane_edge_supported(
                            torus,
                            plane,
                            vertices[a],
                            vertices[b],
                            tolerance,
                        ),
                    };
                    if !supported {
                        demote_trace(candidate, neighbor, "edge with plane neighbour unsupported", vertices[a], vertices[b]);
                        demote.insert(candidate);
                        note(candidate, cause, neighbor);
                    }
                    continue;
                }
                // Only a ruling can be shared exactly with a facet; a
                // sphere has none.
                let supported = match own {
                    Revolve::Cylinder(cylinder) => {
                        edge_is_cylinder_ruling(cylinder, vertices[a], vertices[b], tolerance)
                    }
                    Revolve::Cone(cone) => {
                        edge_is_cone_ruling(cone, vertices[a], vertices[b], tolerance)
                    }
                    Revolve::Sphere(_) | Revolve::Torus(_) => false,
                };
                if !supported {
                    demote_trace(candidate, neighbor, "edge with non-plane neighbour is not a ruling", vertices[a], vertices[b]);
                    demote.insert(candidate);
                    note(candidate, DemotionCause::DemotedNeighbourNotRuling, neighbor);
                }
            }
        }
        if demote.is_empty() {
            break;
        }
        for region in demote {
            retained.remove(&region);
            if let Some(&(cause, neighbor)) = round_causes.get(&region) {
                causes.entry(region).or_insert((cause, neighbor, round));
            }
        }
    }

    let demoted = original_cylinders
        .keys()
        .chain(original_cones.keys())
        .chain(original_spheres.keys())
        .chain(original_tori.keys())
        .copied()
        .filter(|id| !retained.contains(id))
        .collect::<BTreeSet<_>>();
    let mut region_triangles = HashMap::<u32, usize>::new();
    for region in triangle_regions {
        if demoted.contains(region) {
            *region_triangles.entry(*region).or_default() += 1;
        }
    }
    // Rounds only ever demote after their neighbours, so the roots of a
    // recursive chain resolve in round order.
    let mut by_round = demoted.iter().copied().collect::<Vec<_>>();
    by_round.sort_by_key(|id| causes.get(id).map_or(0, |&(_, _, round)| round));
    let mut roots = HashMap::<u32, DemotionCause>::new();
    let mut demotions = Vec::with_capacity(demoted.len());
    for region in by_round {
        let kind = if original_cylinders.contains_key(&region) {
            DemotedKind::Cylinder
        } else if original_cones.contains_key(&region) {
            DemotedKind::Cone
        } else if original_spheres.contains_key(&region) {
            DemotedKind::Sphere
        } else {
            DemotedKind::Torus
        };
        let (cause, neighbour, round, message) = if let Some((cause, message)) = forced_demotions.get(&region) {
            (*cause, None, 0, Some(message.clone()))
        } else {
            let &(cause, neighbour, round) = causes
                .get(&region)
                .ok_or_else(|| format!("hybrid BREP: region {region} demoted without a cause"))?;
            (cause, neighbour, round, None)
        };
        let root_cause = match (cause, neighbour) {
            (DemotionCause::DemotedNeighbourNotRuling, Some(neighbour)) => {
                roots.get(&neighbour).copied().unwrap_or(cause)
            }
            _ => cause,
        };
        roots.insert(region, root_cause);
        demotions.push(HybridDemotion {
            region,
            kind,
            triangles: region_triangles.get(&region).copied().unwrap_or(0),
            cause,
            root_cause,
            neighbour,
            round,
            message,
        });
    }
    demotions.sort_by_key(|demotion| demotion.region);
    let mut planes = HashMap::new();
    let mut cylinders = HashMap::new();
    let mut cones = HashMap::new();
    let mut spheres = HashMap::new();
    let mut tori = HashMap::new();
    let mut original_to_local = HashMap::<u32, u32>::new();
    let mut local_to_original = HashMap::<u32, u32>::new();
    let mut next = 0_u32;
    for region in regions {
        match region.carrier {
            RegionCarrier::Plane { origin, normal } => {
                original_to_local.insert(region.id, next);
                local_to_original.insert(next, region.id);
                planes.insert(next, PlaneInfo { origin, normal });
                next += 1;
            }
            RegionCarrier::Cylinder { .. } if retained.contains(&region.id) => {
                original_to_local.insert(region.id, next);
                local_to_original.insert(next, region.id);
                cylinders.insert(next, original_cylinders[&region.id]);
                next += 1;
            }
            RegionCarrier::Cone { .. } if retained.contains(&region.id) => {
                original_to_local.insert(region.id, next);
                local_to_original.insert(next, region.id);
                cones.insert(next, original_cones[&region.id]);
                next += 1;
            }
            RegionCarrier::Sphere { .. } if retained.contains(&region.id) => {
                original_to_local.insert(region.id, next);
                local_to_original.insert(next, region.id);
                spheres.insert(next, original_spheres[&region.id]);
                next += 1;
            }
            RegionCarrier::Torus { .. } if retained.contains(&region.id) => {
                original_to_local.insert(region.id, next);
                local_to_original.insert(next, region.id);
                tori.insert(next, original_tori[&region.id]);
                next += 1;
            }
            _ => {}
        }
    }

    if std::env::var("BREP_DEBUG_HYBRID").is_ok() {
        for (&local, cylinder) in &cylinders {
            eprintln!(
                "[hybrid] region {local} (original {}): cylinder r={:.4} axis=({:+.4},{:+.4},{:+.4}) origin=({:.3},{:.3},{:.3}) sense={}",
                local_to_original[&local],
                cylinder.radius,
                cylinder.axis.x, cylinder.axis.y, cylinder.axis.z,
                cylinder.origin.x, cylinder.origin.y, cylinder.origin.z,
                cylinder.sense
            );
        }
    }
    let fallback = triangle_regions
        .iter()
        .map(|original| !original_to_local.contains_key(original))
        .collect::<Vec<_>>();
    let mut neighbors = vec![Vec::<usize>::new(); triangles.len()];
    for incident in incidence.values() {
        let first = incident[0];
        let second = incident[1];
        if fallback[first] && fallback[second] {
            neighbors[first].push(second);
            neighbors[second].push(first);
        }
    }

    // Check every member against one fixed seed carrier. Pairwise-only
    // transitive merging could otherwise accumulate a long sequence of tiny
    // creases into a patch that is not actually planar.
    let mut fallback_component = vec![None; triangles.len()];
    let mut fallback_components = 0_usize;
    for seed in 0..triangles.len() {
        if !fallback[seed] || fallback_component[seed].is_some() {
            continue;
        }
        let component = fallback_components;
        fallback_components += 1;
        fallback_component[seed] = Some(component);
        let mut queue = VecDeque::from([seed]);
        while let Some(current) = queue.pop_front() {
            for &neighbor in &neighbors[current] {
                if fallback_component[neighbor].is_none()
                    && facets_are_coplanar(
                        triangles[seed],
                        triangles[neighbor],
                        vertices,
                        tolerance,
                    )?
                {
                    fallback_component[neighbor] = Some(component);
                    queue.push_back(neighbor);
                }
            }
        }
    }

    let mut fallback_to_local = HashMap::<usize, u32>::new();
    let mut local_triangle_regions = Vec::with_capacity(triangles.len());
    for (triangle_id, &original) in triangle_regions.iter().enumerate() {
        if let Some(&local) = original_to_local.get(&original) {
            local_triangle_regions.push(local);
        } else {
            let component = fallback_component[triangle_id]
                .ok_or_else(|| "hybrid BREP: fallback component was not assigned".to_owned())?;
            let local = if let Some(&local) = fallback_to_local.get(&component) {
                local
            } else {
                let local = next;
                next += 1;
                planes.insert(local, facet_plane(triangles[triangle_id], vertices)?);
                fallback_to_local.insert(component, local);
                local
            };
            local_triangle_regions.push(local);
        }
    }
    let analytic_plane_triangles = triangle_regions
        .iter()
        .filter(|region| original_planes.contains_key(region))
        .count();
    let analytic_cylinder_triangles = triangle_regions
        .iter()
        .filter(|region| retained.contains(region) && original_cylinders.contains_key(region))
        .count();
    let analytic_cone_triangles = triangle_regions
        .iter()
        .filter(|region| retained.contains(region) && original_cones.contains_key(region))
        .count();
    let analytic_sphere_triangles = triangle_regions
        .iter()
        .filter(|region| retained.contains(region) && original_spheres.contains_key(region))
        .count();
    let analytic_torus_triangles = triangle_regions
        .iter()
        .filter(|region| retained.contains(region) && original_tori.contains_key(region))
        .count();
    let demoted_triangles = triangle_regions
        .iter()
        .filter(|region| demoted.contains(region))
        .count();
    // [`HybridSwitches::crease_cut`]: counted per source triangle, faceted
    // when any piece of it is.
    let faceted_triangles = match parent {
        Some(parent) => fallback
            .iter()
            .zip(parent)
            .filter(|(&faceted, _)| faceted)
            .map(|(_, &source)| source)
            .collect::<BTreeSet<_>>()
            .len(),
        None => {
            triangles.len()
                - analytic_plane_triangles
                - analytic_cylinder_triangles
                - analytic_cone_triangles
                - analytic_sphere_triangles
                - analytic_torus_triangles
        }
    };
    let stats = HybridBrepStats {
        total_faces: next as usize,
        analytic_plane_faces: original_planes.len(),
        analytic_plane_triangles,
        analytic_cylinder_faces: original_cylinders
            .keys()
            .filter(|id| retained.contains(id))
            .count(),
        analytic_cylinder_triangles,
        analytic_cone_faces: original_cones
            .keys()
            .filter(|id| retained.contains(id))
            .count(),
        analytic_cone_triangles,
        analytic_sphere_faces: original_spheres
            .keys()
            .filter(|id| retained.contains(id))
            .count(),
        analytic_sphere_triangles,
        analytic_torus_faces: original_tori
            .keys()
            .filter(|id| retained.contains(id))
            .count(),
        analytic_torus_triangles,
        faceted_faces: fallback_to_local.len(),
        faceted_triangles,
        demoted_regions: demoted.len(),
        demoted_triangles,
        segmentation_plane_regions: original_planes.len(),
        segmentation_cylinder_regions: original_cylinders.len(),
        segmentation_cone_regions: original_cones.len(),
        segmentation_sphere_regions: original_spheres.len(),
        segmentation_torus_regions: original_tori.len(),
        segmentation_unsupported_regions: regions.len()
            - original_planes.len()
            - original_cylinders.len()
            - original_cones.len()
            - original_spheres.len()
            - original_tori.len(),
        // Filled in by the retry loop from the segmentation's split report.
        split_freeform_regions: 0,
        split_declined_regions: 0,
        split_over_budget_regions: 0,
        split_budget_hit_regions: 0,
        split_band_declined_regions: 0,
        gathered_cone_runs: 0,
        moved_line_ends: 0,
        whole_carrier_scan: Default::default(),
        demotions,
    };
    Ok(LocalRegions {
        triangle_regions: local_triangle_regions,
        planes,
        cylinders,
        cones,
        spheres,
        tori,
        local_to_original,
        count: next as usize,
        stats,
    })
}

fn region_boundaries(
    triangles: &[[usize; 3]],
    regions: &[u32],
    region_count: usize,
    incidence: &BTreeMap<EdgeKey, Vec<usize>>,
) -> Result<(Vec<Chain>, Vec<RegionBoundary>), String> {
    let mut chains = Vec::<Chain>::new();
    let mut chain_by_edges = BTreeMap::<Vec<EdgeKey>, usize>::new();
    let mut boundaries = Vec::with_capacity(region_count);
    for region in 0..region_count as u32 {
        // Directed boundary edges of this region, keyed by their start, with
        // the region triangle that owns each. A vertex normally starts one;
        // where two holes of one face touch at a vertex (two equal-radius
        // crossing cylinders leave the through wall's two branch openings
        // meeting at the tangent points) it starts two, and the walk picks
        // the successor by turning through this region's own triangles
        // around the vertex, which keeps each loop on its own side.
        let mut outgoing = BTreeMap::<usize, Vec<(usize, u32, usize)>>::new();
        for (triangle_id, triangle) in triangles.iter().enumerate() {
            if regions[triangle_id] != region {
                continue;
            }
            for corner in 0..3 {
                let a = triangle[corner];
                let b = triangle[(corner + 1) % 3];
                let incident = &incidence[&edge_key(a, b)];
                let other = incident.iter().copied().find(|&id| id != triangle_id);
                let neighbor = other.map(|id| regions[id]).unwrap_or(u32::MAX);
                if neighbor != region {
                    outgoing.entry(a).or_default().push((b, neighbor, triangle_id));
                }
            }
        }
        // The successor of boundary edge (from -> at) owned by `owner`:
        // rotate around `at` through region triangles until an edge leaving
        // `at` is a boundary edge.
        let successor = |at: usize, owner: usize| -> Option<(usize, u32)> {
            let starts = outgoing.get(&at)?;
            if starts.len() == 1 {
                return Some((starts[0].0, starts[0].1));
            }
            let mut triangle_id = owner;
            for _ in 0..triangles.len() {
                let triangle = triangles[triangle_id];
                let corner = (0..3).find(|&c| triangle[c] == at)?;
                let next = triangle[(corner + 1) % 3];
                if let Some(&(b, neighbor, _)) = starts.iter().find(|&&(b, _, t)| b == next && t == triangle_id) {
                    return Some((b, neighbor));
                }
                let across = incidence[&edge_key(at, next)]
                    .iter()
                    .copied()
                    .find(|&id| id != triangle_id && regions[id] == region)?;
                triangle_id = across;
            }
            None
        };
        let mut owner_of = HashMap::<(usize, usize), usize>::new();
        for (&a, list) in &outgoing {
            for &(b, _, t) in list {
                owner_of.insert((a, b), t);
            }
        }
        let starts = outgoing
            .iter()
            .flat_map(|(&a, list)| list.iter().map(move |&(b, _, _)| (a, b)))
            .collect::<Vec<_>>();
        let mut visited = BTreeSet::<(usize, usize)>::new();
        let mut cycles = Vec::new();
        for (start_a, start_b) in starts {
            if visited.contains(&(start_a, start_b)) {
                continue;
            }
            let mut steps = Vec::<(usize, usize, u32)>::new();
            let (mut current, mut next) = (start_a, start_b);
            let mut neighbor = outgoing[&start_a]
                .iter()
                .find(|&&(b, _, _)| b == start_b)
                .map(|&(_, n, _)| n)
                .unwrap_or(u32::MAX);
            loop {
                if !visited.insert((current, next)) {
                    return Err(format!(
                        "hybrid BREP: region {region} boundary self-crosses"
                    ));
                }
                steps.push((current, next, neighbor));
                let owner = owner_of[&(current, next)];
                let (after, after_neighbor) = successor(next, owner)
                    .ok_or_else(|| format!("hybrid BREP: region {region} boundary walk is open"))?;
                current = next;
                next = after;
                neighbor = after_neighbor;
                if (current, next) == (start_a, start_b) {
                    break;
                }
            }
            let all_one_neighbor = steps.iter().all(|step| step.2 == steps[0].2);
            let mut groups = Vec::<Vec<(usize, usize, u32)>>::new();
            if all_one_neighbor {
                groups.push(steps);
            } else {
                let cut = (0..steps.len())
                    .find(|&index| {
                        steps[index].2 != steps[(index + steps.len() - 1) % steps.len()].2
                    })
                    .unwrap();
                steps.rotate_left(cut);
                let mut begin = 0;
                while begin < steps.len() {
                    let neighbor = steps[begin].2;
                    let mut end = begin + 1;
                    while end < steps.len() && steps[end].2 == neighbor {
                        end += 1;
                    }
                    groups.push(steps[begin..end].to_vec());
                    begin = end;
                }
            }
            let mut traversals = Vec::new();
            for group in groups {
                // `steps` is one boundary cycle. It may be only one of several
                // cycles owned by this region, so comparing against the
                // region-wide outgoing-edge count would misclassify a hole ring.
                let closed = all_one_neighbor;
                let mut chain_vertices = vec![group[0].0];
                chain_vertices.extend(group.iter().map(|step| step.1));
                if closed {
                    chain_vertices.pop();
                }
                let mut key = group
                    .iter()
                    .map(|step| edge_key(step.0, step.1))
                    .collect::<Vec<_>>();
                key.sort_unstable();
                let chain_id = if let Some(&existing) = chain_by_edges.get(&key) {
                    existing
                } else {
                    let id = chains.len();
                    let adjacent = (region.min(group[0].2), region.max(group[0].2));
                    chains.push(Chain {
                        vertices: chain_vertices.clone(),
                        closed,
                        adjacent,
                    });
                    chain_by_edges.insert(key, id);
                    id
                };
                let stored = &chains[chain_id];
                let a = chain_vertices[0];
                let b = chain_vertices[1];
                let position = stored
                    .vertices
                    .iter()
                    .position(|&vertex| vertex == a)
                    .ok_or("hybrid BREP: shared chain start is missing")?;
                // A loop pinned at one vertex (a face boundary touching itself
                // where two openings meet) is stored open with equal ends.
                let pinned = !stored.closed
                    && stored.vertices.len() > 2
                    && stored.vertices.first() == stored.vertices.last();
                let forward = if stored.closed {
                    stored.vertices[(position + 1) % stored.vertices.len()] == b
                } else if position + 1 < stored.vertices.len() && stored.vertices[position + 1] == b
                {
                    true
                } else if position > 0 && stored.vertices[position - 1] == b {
                    false
                } else if pinned && position == 0 && stored.vertices[stored.vertices.len() - 2] == b {
                    false
                } else {
                    return Err("hybrid BREP: shared chain directions disagree".into());
                };
                traversals.push(Traversal {
                    chain: chain_id,
                    forward,
                });
            }
            cycles.push(traversals);
        }
        boundaries.push(RegionBoundary { cycles });
    }
    Ok((chains, boundaries))
}

/// Two planar regions meet along a line, so their chain is one straight
/// edge — unless one of them is a leftover facet whose two edges into the
/// plane bend at a shared vertex. Such a chain becomes one straight edge
/// per bend, and every traversal of it in the region cycles is rewritten in
/// order.
fn split_bent_planar_chains(
    chains: Vec<Chain>,
    boundaries: &mut [RegionBoundary],
    vertices: &[Vec3],
    tolerance: f64,
    is_revolve: impl Fn(u32) -> bool,
) -> Vec<Chain> {
    let mut result = Vec::with_capacity(chains.len());
    let mut replacement: Vec<Vec<usize>> = Vec::with_capacity(chains.len());
    for chain in chains {
        let planar = !is_revolve(chain.adjacent.0) && !is_revolve(chain.adjacent.1);
        let points = chain_points(&chain, vertices);
        if !planar || chain.closed || points.len() < 3 || max_line_deviation(&points) <= tolerance {
            replacement.push(vec![result.len()]);
            result.push(chain);
            continue;
        }
        // Greedy runs of collinear mesh edges.
        let mut pieces = Vec::new();
        let mut start = 0;
        for end in 2..chain.vertices.len() {
            if max_line_deviation(&points[start..=end]) > tolerance {
                pieces.push(result.len());
                result.push(Chain {
                    vertices: chain.vertices[start..end].to_vec(),
                    closed: false,
                    adjacent: chain.adjacent,
                });
                start = end - 1;
            }
        }
        pieces.push(result.len());
        result.push(Chain {
            vertices: chain.vertices[start..].to_vec(),
            closed: false,
            adjacent: chain.adjacent,
        });
        replacement.push(pieces);
    }
    for boundary in boundaries.iter_mut() {
        for cycle in &mut boundary.cycles {
            let mut rewritten = Vec::with_capacity(cycle.len());
            for traversal in cycle.iter() {
                let pieces = &replacement[traversal.chain];
                if traversal.forward {
                    rewritten.extend(pieces.iter().map(|&chain| Traversal {
                        chain,
                        forward: true,
                    }));
                } else {
                    rewritten.extend(pieces.iter().rev().map(|&chain| Traversal {
                        chain,
                        forward: false,
                    }));
                }
            }
            *cycle = rewritten;
        }
    }
    result
}

/// Rewrite every traversal of chain `i` as the traversals of
/// `replacement[i]`, in order (reversed for a backward traversal).
fn rewrite_traversals(boundaries: &mut [RegionBoundary], replacement: &[Vec<usize>]) {
    for boundary in boundaries.iter_mut() {
        for cycle in &mut boundary.cycles {
            let mut rewritten = Vec::with_capacity(cycle.len());
            for traversal in cycle.iter() {
                let pieces = &replacement[traversal.chain];
                if traversal.forward {
                    rewritten.extend(pieces.iter().map(|&chain| Traversal { chain, forward: true }));
                } else {
                    rewritten.extend(pieces.iter().rev().map(|&chain| Traversal { chain, forward: false }));
                }
            }
            *cycle = rewritten;
        }
    }
}

/// Two equal-radius cylinders whose axes cross meet in two half-ellipses,
/// one in each section plane of [`cylinder_pair_section`], joined at the two
/// tangent points `center ± r·(a1 × a2)`. A tessellation rarely puts a vertex
/// exactly there: the mesh edge that cuts the corner has one end on each
/// plane, the closed chain lies in neither, and the builder demoted both
/// walls ("neither a shared ruling nor an equal-radius crossing": the kernel's
/// own crossed cylinders faceted at every chord).
///
/// For such a closed chain the vertex nearest each tangent point is moved
/// ONTO it — the vertex already lies on both cylinders (it is on their
/// intersection curve) and the tangent point does too, so no carrier is left
/// — provided no other chain uses that vertex and the move is shorter than
/// the chain edges beside it. The loop is then split at those two vertices
/// into two open chains, each kept only if all its points lie on one section
/// plane; otherwise nothing is changed.
/// Lane J row 7 (unless BREP_HYBRID_SPLIT_FREEFORM=0): when both crossing
/// walls are in two pieces each, every seam is already an open chain ending
/// at a tangent point, which [`split_crossing_cylinder_chains`] never moves.
/// On a noisy mesh that end sits up to the noise off the point (5e-3 at
/// radius 10) and each section curve misses it by as much. An open
/// cylinder/cylinder chain end within the exact-topology bar of a tangent
/// point of its pair moves onto the point; the point lies on both walls, and
/// every chain through it is a seam of the same crossing.
fn pin_crossing_chain_ends(
    chains: &[Chain],
    vertices: &mut [Vec3],
    cylinders: &HashMap<u32, CylinderCarrier>,
    tolerance: f64,
) {
    for chain in chains {
        if chain.closed || chain.vertices.len() < 2 {
            continue;
        }
        let (Some(&first), Some(&second)) = (cylinders.get(&chain.adjacent.0), cylinders.get(&chain.adjacent.1)) else {
            continue;
        };
        let Some((center, _)) = cylinder_pair_section(first, second, tolerance) else {
            continue;
        };
        let Ok(minor) = first.axis.cross(second.axis).normalized() else {
            continue;
        };
        let radius = 0.5 * (first.radius + second.radius);
        for end in [chain.vertices[0], *chain.vertices.last().unwrap()] {
            for tangent in [center.add(minor.scale(radius)), center.sub(minor.scale(radius))] {
                if vertices[end].sub(tangent).length() <= tolerance * 4.0 {
                    vertices[end] = tangent;
                }
            }
        }
    }
}

/// Two cylinder carriers are pieces of one wall: axes parallel within 0.01,
/// radii and each origin's distance from the other's axis line within the
/// pair's radial bar at `tolerance * 4`.
fn same_cylinder_wall(first: CylinderCarrier, second: CylinderCarrier, tolerance: f64) -> bool {
    let off_line = |c: CylinderCarrier, p: Vec3| {
        let d = p.sub(c.origin);
        d.sub(c.axis.scale(d.dot(c.axis))).length()
    };
    let bar = accepted_radial_bar(tolerance * 4.0, first.deviation.max(second.deviation));
    first.axis.cross(second.axis).length() <= 0.01
        && (first.radius - second.radius).abs() <= bar
        && off_line(first, second.origin) <= bar
        && off_line(second, first.origin) <= bar
}

/// [`HybridSwitches::shared_crossing`]: give the pieces of one cylinder wall
/// that cross the same third cylinder one carrier, so both crossings share
/// their tangent points and section ellipses.
fn share_crossing_wall_carriers(
    cylinders: &mut HashMap<u32, CylinderCarrier>,
    chains: &[Chain],
    tolerance: f64,
) {
    let mut crossed = HashMap::<u32, BTreeSet<u32>>::new();
    for chain in chains {
        let (a, b) = chain.adjacent;
        let (Some(&first), Some(&second)) = (cylinders.get(&a), cylinders.get(&b)) else {
            continue;
        };
        if cylinder_pair_section(first, second, tolerance).is_some() {
            crossed.entry(a).or_default().insert(b);
            crossed.entry(b).or_default().insert(a);
        }
    }
    let mut regions = crossed.keys().copied().collect::<Vec<_>>();
    regions.sort_unstable();
    let mut group = regions.iter().map(|&r| (r, r)).collect::<HashMap<_, _>>();
    fn root(group: &HashMap<u32, u32>, mut r: u32) -> u32 {
        while group[&r] != r {
            r = group[&r];
        }
        r
    }
    for (i, &a) in regions.iter().enumerate() {
        for &b in &regions[i + 1..] {
            if !same_cylinder_wall(cylinders[&a], cylinders[&b], tolerance) || crossed[&a].is_disjoint(&crossed[&b]) {
                continue;
            }
            let (ra, rb) = (root(&group, a), root(&group, b));
            group.insert(ra.max(rb), ra.min(rb));
        }
    }
    let mut members = BTreeMap::<u32, Vec<u32>>::new();
    for &r in &regions {
        members.entry(root(&group, r)).or_default().push(r);
    }
    for pieces in members.values().filter(|pieces| pieces.len() > 1) {
        let lead = cylinders[&pieces[0]];
        let n = pieces.len() as f64;
        let Ok(axis) = pieces
            .iter()
            .map(|r| {
                let a = cylinders[r].axis;
                if a.dot(lead.axis) >= 0.0 { a } else { a.scale(-1.0) }
            })
            .fold(Vec3::default(), |sum, a| sum.add(a))
            .normalized()
        else {
            continue;
        };
        // Each origin moved along the mean axis to the lead's station, then
        // averaged.
        let origin = pieces
            .iter()
            .map(|r| {
                let o = cylinders[r].origin;
                o.sub(axis.scale(o.sub(lead.origin).dot(axis)))
            })
            .fold(Vec3::default(), |sum, o| sum.add(o))
            .scale(1.0 / n);
        let radius = pieces.iter().map(|r| cylinders[r].radius).sum::<f64>() / n;
        let deviation = pieces.iter().map(|r| cylinders[r].deviation).fold(0.0, f64::max);
        for r in pieces {
            let piece = cylinders.get_mut(r).unwrap();
            let axis = if piece.axis.dot(axis) >= 0.0 { axis } else { axis.scale(-1.0) };
            *piece = CylinderCarrier { origin, axis, radius, deviation, ..*piece };
        }
    }
}

fn split_crossing_cylinder_chains(
    chains: Vec<Chain>,
    boundaries: &mut [RegionBoundary],
    vertices: &mut [Vec3],
    cylinders: &HashMap<u32, CylinderCarrier>,
    tolerance: f64,
) -> Vec<Chain> {
    let mut uses = HashMap::<usize, usize>::new();
    for chain in &chains {
        for &vertex in &chain.vertices {
            *uses.entry(vertex).or_default() += 1;
        }
    }
    let mut result = Vec::with_capacity(chains.len());
    let mut replacement: Vec<Vec<usize>> = Vec::with_capacity(chains.len());
    let mut changed = false;
    for chain in chains {
        let split = (|| -> Option<(Vec<Chain>, Vec<(usize, Vec3)>)> {
            // A loop pinned at one vertex is stored open with equal ends;
            // split it as the loop of its distinct vertices, keeping the pin
            // as the first piece's start.
            let pinned = !chain.closed
                && chain.vertices.len() > 3
                && chain.vertices.first() == chain.vertices.last();
            if !(chain.closed || pinned) || chain.vertices.len() < 4 {
                return None;
            }
            let loop_vertices = if pinned {
                chain.vertices[..chain.vertices.len() - 1].to_vec()
            } else {
                chain.vertices.clone()
            };
            let first = *cylinders.get(&chain.adjacent.0)?;
            let second = *cylinders.get(&chain.adjacent.1)?;
            let (center, normals) = cylinder_pair_section(first, second, tolerance)?;
            let bar = tolerance * 4.0;
            let plane_bar = cylinder_pair_section_bar(first, second, tolerance);
            let points: Vec<Vec3> = loop_vertices.iter().map(|&id| vertices[id]).collect();
            let on = |point: Vec3, k: usize| point.sub(center).dot(normals[k]).abs() <= plane_bar;
            if points.iter().all(|&p| on(p, 0)) || points.iter().all(|&p| on(p, 1)) {
                return None;
            }
            let minor = first.axis.cross(second.axis).normalized().ok()?;
            let radius = 0.5 * (first.radius + second.radius);
            let count = loop_vertices.len();
            let mut corners = Vec::new();
            for tangent in [center.add(minor.scale(radius)), center.sub(minor.scale(radius))] {
                let (index, distance) = points
                    .iter()
                    .enumerate()
                    .map(|(i, p)| (i, p.sub(tangent).length()))
                    .min_by(|a, b| a.1.total_cmp(&b.1))?;
                let before = points[(index + count - 1) % count].sub(points[index]).length();
                let after = points[(index + 1) % count].sub(points[index]).length();
                // Already on the tangent point: nothing moves, so other
                // chains through it (the pin) are unaffected. Lane J row 7
                // (unless BREP_HYBRID_SPLIT_FREEFORM=0): on a noisy mesh "on"
                // means within the noise-raised bar, and a pin 5e-3 off the
                // point at radius 10 left every section curve that gap from
                // it, past topology validation; the pin then moves onto the
                // point (below), which every chain through it shares.
                let exact = distance <= bar;
                if !exact && (uses.get(&loop_vertices[index]) != Some(&1) || distance >= before.max(after)) {
                    // Under the pair-section bar a corner cut may end up to
                    // sqrt(2 r bar) from a tangent point
                    // (`cylinder_pair_corner_cut`), so its end moves onto the
                    // point from that far: on a chordal crossing (walls fitted
                    // at 1.7e-2) the corner sat 6.5e-2 off the fitted point,
                    // as far as its neighbours, and the loop stayed closed.
                    let reach = (2.0 * radius * plane_bar).sqrt();
                    if !(first.pair_section_bar || second.pair_section_bar)
                        || uses.get(&loop_vertices[index]) != Some(&1)
                        || distance > reach
                    {
                        return None;
                    }
                }
                corners.push((index, tangent));
            }
            let (i, j) = (corners[0].0.min(corners[1].0), corners[0].0.max(corners[1].0));
            if i == j || j - i < 2 || count - (j - i) < 2 {
                return None;
            }
            let mut moved = points.clone();
            for &(index, tangent) in &corners {
                moved[index] = tangent;
            }
            let one_plane = |range: &[Vec3]| range.iter().all(|&p| on(p, 0)) || range.iter().all(|&p| on(p, 1));
            let mut wrap = moved[j..].to_vec();
            wrap.extend_from_slice(&moved[..=i]);
            if !one_plane(&moved[i..=j]) || !one_plane(&wrap) {
                return None;
            }
            // A pinned chain is traversed from its pin (stored vertex 0), so
            // the pin must be one of the two corners.
            if pinned && i != 0 {
                return None;
            }
            let mut tail = loop_vertices[j..].to_vec();
            tail.extend_from_slice(&loop_vertices[..=i]);
            // Traversal order: from stored vertex 0 (= corner i when i == 0)
            // the loop runs i..=j, then j.. wrapping back to i.
            let pieces = vec![
                Chain { vertices: loop_vertices[i..=j].to_vec(), closed: false, adjacent: chain.adjacent },
                Chain { vertices: tail, closed: false, adjacent: chain.adjacent },
            ];
            let pin_moves = split_freeform_enabled();
            let moves = corners
                .iter()
                .filter(|&&(index, t)| {
                    let off = points[index].sub(t).length();
                    off > bar || (pin_moves && off > 0.0)
                })
                .map(|&(index, t)| (loop_vertices[index], t))
                .collect();
            Some((pieces, moves))
        })();
        match split {
            Some((pieces, moves)) => {
                for (vertex, tangent) in moves {
                    vertices[vertex] = tangent;
                }
                let start = result.len();
                replacement.push((start..start + pieces.len()).collect());
                result.extend(pieces);
                changed = true;
            }
            None => {
                replacement.push(vec![result.len()]);
                result.push(chain);
            }
        }
    }
    if changed {
        rewrite_traversals(boundaries, &replacement);
    }
    result
}

fn chain_points(chain: &Chain, vertices: &[Vec3]) -> Vec<Vec3> {
    chain.vertices.iter().map(|&id| vertices[id]).collect()
}

fn snap_cylinder_axes(
    cylinders: &mut HashMap<u32, CylinderCarrier>,
    planes: &HashMap<u32, PlaneInfo>,
    chains: &[Chain],
    vertices: &[Vec3],
    tolerance: f64,
) -> Result<(), (u32, String)> {
    let mut snapped = HashMap::<u32, Vec<Vec3>>::new();
    for (&region, cylinder) in cylinders.iter() {
        let mut candidates = Vec::new();
        for chain in chains.iter().filter(|chain| {
            chain.closed && (chain.adjacent.0 == region || chain.adjacent.1 == region)
        }) {
            let neighbor = if chain.adjacent.0 == region {
                chain.adjacent.1
            } else {
                chain.adjacent.0
            };
            let Some(plane) = planes.get(&neighbor) else {
                continue;
            };
            if plane.normal.cross(cylinder.axis).length() > 0.01 {
                continue;
            }
            let points = chain_points(chain, vertices);
            let spread = points
                .iter()
                .map(|point| point.sub(plane.origin).dot(plane.normal).abs())
                .fold(0.0_f64, f64::max);
            if spread <= tolerance {
                let axis = if plane.normal.dot(cylinder.axis) >= 0.0 {
                    plane.normal
                } else {
                    plane.normal.scale(-1.0)
                };
                candidates.push(axis.normalized().map_err(|error| (region, error))?);
            }
        }
        snapped.insert(region, candidates);
    }
    // Lane J row 7 (unless BREP_HYBRID_SPLIT_FREEFORM=0): regions that carry
    // one carrier bit for bit are pieces of one wall (the freeform split
    // writes each edge-connected piece of a carrier as its own region) and
    // take one axis: the walls of crossing cylinders meet at tangent points
    // shared by every piece, so pieces snapped to different noisy cap normals
    // imply different tangent points and the section curves miss them.
    let shared = split_freeform_enabled();
    let original = cylinders.clone();
    let mut regions: Vec<u32> = cylinders.keys().copied().collect();
    regions.sort_unstable();
    for &region in &regions {
        let cylinder = original[&region];
        let group: Vec<u32> = if shared {
            regions
                .iter()
                .copied()
                .filter(|other| {
                    let o = original[other];
                    let same = |a: Vec3, b: Vec3| a.x == b.x && a.y == b.y && a.z == b.z;
                    same(o.origin, cylinder.origin) && same(o.axis, cylinder.axis) && o.radius == cylinder.radius
                })
                .collect()
        } else {
            vec![region]
        };
        let candidates: Vec<Vec3> = group.iter().flat_map(|member| snapped[member].iter().copied()).collect();
        if let Some(&axis) = candidates.first() {
            // Independent plane fits to rounded STL rings need not have
            // bit-identical normals. Bound their disagreement by the
            // positional error it induces around this cylinder's rim,
            // using the same tolerance as the shared topology checks.
            if candidates
                .iter()
                .any(|other| cylinder.radius * other.cross(axis).length() > tolerance)
            {
                return Err((
                    region,
                    format!("hybrid BREP: cylinder {region} has inconsistent ring normals"),
                ));
            }
            let axis = if group.len() > 1 {
                candidates
                    .iter()
                    .fold(Vec3::new(0.0, 0.0, 0.0), |sum, &c| sum.add(c))
                    .normalized()
                    .map_err(|error| (region, error))?
            } else {
                axis
            };
            if let Some(cylinder) = cylinders.get_mut(&region) {
                cylinder.axis = axis;
            }
        }
    }
    Ok(())
}

fn snap_cone_carriers(
    cones: &mut HashMap<u32, ConeCarrier>,
    cylinders: &HashMap<u32, CylinderCarrier>,
    planes: &HashMap<u32, PlaneInfo>,
    chains: &[Chain],
    vertices: &[Vec3],
    tolerance: f64,
    ring_fit_center: bool,
    two_rings: bool,
) -> Result<(), (u32, String)> {
    for (&region, cone) in cones.iter_mut() {
        let relevant = chains
            .iter()
            .filter(|chain| chain.adjacent.0 == region || chain.adjacent.1 == region)
            .collect::<Vec<_>>();
        let mut snapped = None::<(Vec3, Vec3)>;

        // A cylinder/cone join is a shared circle. The cylinder fixes the
        // coaxial frame and the cone apex then follows from its opening angle.
        for chain in &relevant {
            let neighbor = if chain.adjacent.0 == region {
                chain.adjacent.1
            } else {
                chain.adjacent.0
            };
            let Some(cylinder) = cylinders.get(&neighbor).copied() else {
                continue;
            };
            let mut axis = cylinder.axis;
            if axis.dot(cone.axis) < 0.0 {
                axis = axis.scale(-1.0);
            }
            let points = chain_points(chain, vertices);
            let center_station = points
                .iter()
                .map(|&point| point.sub(cylinder.origin).dot(axis))
                .sum::<f64>()
                / points.len() as f64;
            let center = cylinder.origin.add(axis.scale(center_station));

            let mut rings = Vec::<(f64, f64)>::new();
            for boundary_chain in &relevant {
                let boundary_points = chain_points(boundary_chain, vertices);
                let stations = boundary_points
                    .iter()
                    .map(|&point| point.sub(cylinder.origin).dot(axis))
                    .collect::<Vec<_>>();
                let low_station = stations.iter().copied().fold(f64::INFINITY, f64::min);
                let high_station = stations.iter().copied().fold(f64::NEG_INFINITY, f64::max);
                if high_station - low_station > tolerance * 4.0 {
                    continue;
                }
                let station = stations.iter().sum::<f64>() / stations.len() as f64;
                let neighbor = if boundary_chain.adjacent.0 == region {
                    boundary_chain.adjacent.1
                } else {
                    boundary_chain.adjacent.0
                };
                let radius = if cylinders.contains_key(&neighbor) {
                    cylinder.radius
                } else {
                    boundary_points
                        .iter()
                        .map(|&point| {
                            let delta = point.sub(cylinder.origin);
                            delta.sub(axis.scale(delta.dot(axis))).length()
                        })
                        .sum::<f64>()
                        / boundary_points.len() as f64
                };
                rings.push((station, radius));
            }
            rings.sort_by(|a, b| a.0.total_cmp(&b.0));
            if let (Some(&(s0, r0)), Some(&(s1, r1))) = (rings.first(), rings.last()) {
                let slope = (r1 - r0) / (s1 - s0);
                if slope.is_finite() && slope > 1.0e-6 {
                    let apex_station = s0 - r0 / slope;
                    cone.apex = cylinder.origin.add(axis.scale(apex_station));
                    cone.axis = axis;
                    cone.half_angle = slope.atan();
                    snapped = Some((cone.apex, axis));
                    break;
                }
            }

            // Short cone patches fit their apex poorly from tessellated
            // normals. Their two side planes, however, contain the exact cone
            // apex. Intersect those planes with the coaxial line fixed by the
            // adjacent cylinder, then derive the half-angle from the common
            // cylinder/cone ring.
            let mut apex_stations = Vec::new();
            for side_chain in &relevant {
                let side_neighbor = if side_chain.adjacent.0 == region {
                    side_chain.adjacent.1
                } else {
                    side_chain.adjacent.0
                };
                let Some(side_plane) = planes.get(&side_neighbor) else {
                    continue;
                };
                let side_points = chain_points(side_chain, vertices);
                let side_spread = side_points
                    .iter()
                    .map(|&point| point.sub(cylinder.origin).dot(axis))
                    .fold((f64::INFINITY, f64::NEG_INFINITY), |(low, high), value| {
                        (low.min(value), high.max(value))
                    });
                if side_spread.1 - side_spread.0 <= tolerance * 4.0 {
                    continue;
                }
                let normal = side_plane
                    .normal
                    .normalized()
                    .map_err(|error| (region, error))?;
                let denominator = normal.dot(axis);
                if denominator.abs() <= 1.0e-8 {
                    continue;
                }
                apex_stations
                    .push(normal.dot(side_plane.origin.sub(cylinder.origin)) / denominator);
            }
            let side_solution = if !apex_stations.is_empty() {
                let apex_station = apex_stations.iter().sum::<f64>() / apex_stations.len() as f64;
                let axial_radius = center_station - apex_station;
                if axial_radius <= tolerance {
                    None
                } else {
                    Some((
                        cylinder.origin.add(axis.scale(apex_station)),
                        (cylinder.radius / axial_radius).atan(),
                    ))
                }
            } else {
                None
            };
            // Accept a side-plane apex only when it corroborates the fitted
            // opening angle. Otherwise those planes cut the cone in genuine
            // oblique conics and do not contain its apex.
            let (apex, half_angle) = match side_solution {
                Some((apex, angle)) if (angle - cone.half_angle).abs() <= 0.01 => (apex, angle),
                _ => (
                    center.sub(axis.scale(cylinder.radius / cone.half_angle.tan())),
                    cone.half_angle,
                ),
            };
            cone.half_angle = half_angle;
            snapped = Some((apex, axis));
            break;
        }

        // Otherwise a ring in an axis-normal plane supplies an equally exact
        // frame. This covers truncated cones without an adjacent cylinder.
        if snapped.is_none() {
            // Every ring that snaps, in chain order: (axis, centre, radius,
            // points).
            let mut rings = Vec::<(Vec3, Vec3, f64, Vec<Vec3>)>::new();
            for chain in relevant.iter().filter(|chain| chain.closed) {
                let neighbor = if chain.adjacent.0 == region {
                    chain.adjacent.1
                } else {
                    chain.adjacent.0
                };
                let Some(plane) = planes.get(&neighbor).copied() else {
                    continue;
                };
                let mut axis = plane.normal.normalized().map_err(|error| (region, error))?;
                if axis.dot(cone.axis) < 0.0 {
                    axis = axis.scale(-1.0);
                }
                // A fitted axis up to RING_SNAP_TILT off the normal snaps
                // only when the ring is a circle about it within the bar.
                let tilt = axis.cross(cone.axis).length();
                if tilt > if ring_fit_center { RING_SNAP_TILT } else { 0.01 } {
                    continue;
                }
                let points = chain_points(chain, vertices);
                let mean = points
                    .iter()
                    .copied()
                    .fold(Vec3::new(0.0, 0.0, 0.0), Vec3::add)
                    .scale(1.0 / points.len() as f64);
                let center = mean.sub(axis.scale(mean.sub(plane.origin).dot(axis)));
                // A ring missing some of its vertices has its mean off the axis.
                let center = if ring_fit_center || two_rings {
                    ring_circle_center(&points, center, axis).unwrap_or(center)
                } else {
                    center
                };
                let radius = points
                    .iter()
                    .map(|&point| {
                        let delta = point.sub(center);
                        delta.sub(axis.scale(delta.dot(axis))).length()
                    })
                    .sum::<f64>()
                    / points.len() as f64;
                let bar = accepted_radial_bar(tolerance * 4.0, cone.deviation);
                if tilt > 0.01
                    && points.iter().any(|&point| {
                        let delta = point.sub(center);
                        (delta.sub(axis.scale(delta.dot(axis))).length() - radius).abs() > bar
                    })
                {
                    continue;
                }
                if radius > tolerance {
                    rings.push((axis, center, radius, points));
                    if !two_rings {
                        break;
                    }
                }
            }
            // [`HybridSwitches::frustum_two_rings`]: a cone between two
            // parallel plane rings takes its axis through their centres (a
            // jittered cap's normal tilts by more than the centres move) and
            // its opening from both radii.
            let single = rings.first().map(|(axis, center, radius, _)| {
                (center.sub(axis.scale(radius / cone.half_angle.tan())), *axis)
            });
            // The first ring's frame stands when it already holds the
            // second ring within the bar the ring edge is built to.
            let bar = accepted_radial_bar(tolerance, cone.deviation);
            let second_holds = match (single, rings.get(1)) {
                (Some((apex, axis)), Some((_, _, _, points))) => {
                    let snapped = ConeCarrier { apex, axis, ..*cone };
                    points.iter().all(|&point| {
                        let distance = cone_radial(snapped, point).length() * cone.half_angle.cos()
                            - cone_station(snapped, point) * cone.half_angle.sin();
                        distance.abs() <= bar
                    })
                }
                _ => true,
            };
            let pair = match rings.as_slice() {
                [(axis, c0, r0, _), (other, c1, r1, _), ..]
                    if two_rings && !second_holds && axis.cross(*other).length() <= 0.01 =>
                {
                    let span = c1.sub(*c0).length();
                    c1.sub(*c0)
                        .normalized()
                        .ok()
                        .filter(|line| span > tolerance && line.cross(*axis).length() <= 0.01)
                        .and_then(|line| {
                            let slope = (r1 - r0) / span;
                            let apex = c0.sub(line.scale(r0 / slope));
                            let half_angle = slope.abs().atan();
                            let direction = c0.sub(apex).normalized().ok()?;
                            (slope.is_finite() && (half_angle - cone.half_angle).abs() <= 0.05)
                                .then_some((apex, direction, half_angle))
                        })
                }
                _ => None,
            };
            if let Some((apex, direction, half_angle)) = pair {
                cone.half_angle = half_angle;
                snapped = Some((apex, direction));
            } else if single.is_some() {
                snapped = single;
            }
        }

        if let Some((apex, axis)) = snapped {
            cone.apex = apex;
            cone.axis = axis;
        }
    }
    Ok(())
}

/// [`HybridSwitches::pointed_cone_apex`]: the centre of the least-squares
/// circle x² + y² + d x + e y + f = 0 through `points` in the plane through
/// `center` normal to `axis`. `None` when the points do not fix one.
fn ring_circle_center(points: &[Vec3], center: Vec3, axis: Vec3) -> Option<Vec3> {
    let helper = if axis.x.abs() < 0.9 { Vec3::new(1.0, 0.0, 0.0) } else { Vec3::new(0.0, 1.0, 0.0) };
    let u = axis.cross(helper).normalized().ok()?;
    let v = axis.cross(u);
    let mut normal = [[0.0f64; 4]; 3];
    for &point in points {
        let delta = point.sub(center);
        let (x, y) = (delta.dot(u), delta.dot(v));
        let row = [x, y, 1.0];
        let rhs = -(x * x + y * y);
        for i in 0..3 {
            for j in 0..3 {
                normal[i][j] += row[i] * row[j];
            }
            normal[i][3] += row[i] * rhs;
        }
    }
    let det = |m: [[f64; 3]; 3]| {
        m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
    };
    let matrix = |column: Option<usize>| {
        let mut m = [[0.0; 3]; 3];
        for i in 0..3 {
            for j in 0..3 {
                m[i][j] = if Some(j) == column { normal[i][3] } else { normal[i][j] };
            }
        }
        m
    };
    let whole = det(matrix(None));
    if !whole.is_normal() {
        return None;
    }
    let (d, e) = (det(matrix(Some(0))) / whole, det(matrix(Some(1))) / whole);
    let fitted = center.add(u.scale(-0.5 * d)).add(v.scale(-0.5 * e));
    fitted.x.is_finite().then_some(fitted)
}

/// [`HybridSwitches::revolve_ends_first`]: the open chains on one cylinder
/// or cone whose stations each hold within `tolerance` of their mean are
/// arcs of a ring at that station; arcs of one region whose mean stations
/// lie within `tolerance` of each other are one ring and share the
/// point-weighted mean station (as the surface parameter v), so consecutive
/// arcs end at one point.
fn common_arc_stations(
    chains: &[Chain],
    vertices: &[Vec3],
    cylinders: &HashMap<u32, CylinderInfo>,
    cones: &HashMap<u32, ConeInfo>,
    tolerance: f64,
) -> HashMap<usize, f64> {
    // Per region: (chain, mean v, point count, mean station).
    let mut arcs = BTreeMap::<u32, Vec<(usize, f64, usize, f64)>>::new();
    for (index, chain) in chains.iter().enumerate() {
        if chain.closed {
            continue;
        }
        for region in [chain.adjacent.0, chain.adjacent.1] {
            let (stations, vs): (Vec<f64>, Vec<f64>) = if let Some(cylinder) = cylinders.get(&region) {
                chain_points(chain, vertices)
                    .iter()
                    .map(|&p| {
                        let s = station(cylinder.carrier, p);
                        (s, (s - cylinder.base_station) / cylinder.height)
                    })
                    .unzip()
            } else if let Some(cone) = cones.get(&region) {
                chain_points(chain, vertices)
                    .iter()
                    .map(|&p| {
                        let s = cone_station(cone.carrier, p);
                        (s, (s - cone.base_station) / cone.height)
                    })
                    .unzip()
            } else {
                continue;
            };
            let mean = stations.iter().sum::<f64>() / stations.len() as f64;
            if stations.iter().any(|s| (s - mean).abs() > tolerance) {
                continue;
            }
            let v = vs.iter().sum::<f64>() / vs.len() as f64;
            arcs.entry(region).or_default().push((index, v, vs.len(), mean));
        }
    }
    let mut common = HashMap::new();
    for (_, mut list) in arcs {
        list.sort_by(|a, b| a.3.total_cmp(&b.3));
        let mut start = 0;
        while start < list.len() {
            let mut end = start + 1;
            while end < list.len() && list[end].3 - list[end - 1].3 <= tolerance {
                end += 1;
            }
            if end - start >= 2 {
                let group = &list[start..end];
                let weight: usize = group.iter().map(|arc| arc.2).sum();
                let v = group.iter().map(|arc| arc.1 * arc.2 as f64).sum::<f64>() / weight as f64;
                for arc in group {
                    common.insert(arc.0, v);
                }
            }
            start = end;
        }
    }
    common
}

/// [`HybridSwitches::plane_corners`]: every chain end where only plane
/// regions meet moves to the nearest point on all of their planes (the
/// corner three independent planes share, the nearest point of the line two
/// share, or the foot on one), when that move is within `tolerance`. A plane
/// face then holds its own boundary, and a line between two planes joins
/// points on both.
fn place_plane_corners(vertices: &mut [Vec3], chains: &[Chain], planes: &HashMap<u32, PlaneInfo>, tolerance: f64) {
    let mut incident = BTreeMap::<usize, BTreeSet<u32>>::new();
    for chain in chains {
        let ends = [chain.vertices.first(), chain.vertices.last()];
        for &vertex in ends.into_iter().flatten() {
            incident.entry(vertex).or_default().extend([chain.adjacent.0, chain.adjacent.1]);
        }
    }
    for (vertex, regions) in incident {
        let Some(carriers) = regions.iter().map(|region| planes.get(region)).collect::<Option<Vec<_>>>() else {
            continue;
        };
        // Independent constraints n . p = n . o, kept while each adds a
        // direction at least 0.1 (sine) off the span of those before it.
        let mut rows: Vec<(Vec3, f64)> = Vec::new();
        let mut span: Vec<Vec3> = Vec::new();
        for plane in carriers {
            let Ok(normal) = plane.normal.normalized() else { continue };
            let residual = span.iter().fold(normal, |n, &basis| n.sub(basis.scale(n.dot(basis))));
            if rows.len() == 3 || residual.length() < 0.1 {
                continue;
            }
            span.push(residual.scale(1.0 / residual.length()));
            rows.push((normal, normal.dot(plane.origin)));
        }
        if rows.is_empty() {
            continue;
        }
        // p = v - N^T (N N^T)^-1 (N v - d): the nearest point satisfying
        // every kept row.
        let point = vertices[vertex];
        let k = rows.len();
        let mut gram = [[0.0_f64; 3]; 3];
        let mut excess = [0.0_f64; 3];
        for i in 0..k {
            for j in 0..k {
                gram[i][j] = rows[i].0.dot(rows[j].0);
            }
            excess[i] = rows[i].0.dot(point) - rows[i].1;
        }
        let Some(weights) = solve_small(&gram, &excess, k) else { continue };
        let placed = (0..k).fold(point, |p, i| p.sub(rows[i].0.scale(weights[i])));
        if placed.sub(point).length() <= tolerance {
            vertices[vertex] = placed;
        }
    }
}

/// The solution of the leading `k` x `k` system `a x = b` (Gaussian
/// elimination with partial pivoting), `None` when it is singular.
fn solve_small(a: &[[f64; 3]; 3], b: &[f64; 3], k: usize) -> Option<[f64; 3]> {
    let mut m = [[0.0_f64; 4]; 3];
    for i in 0..k {
        m[i][..k].copy_from_slice(&a[i][..k]);
        m[i][3] = b[i];
    }
    for column in 0..k {
        let pivot = (column..k).max_by(|&x, &y| m[x][column].abs().total_cmp(&m[y][column].abs()))?;
        if m[pivot][column].abs() < 1.0e-12 {
            return None;
        }
        m.swap(column, pivot);
        for row in 0..k {
            if row != column {
                let factor = m[row][column] / m[column][column];
                for c in column..4 {
                    m[row][c] -= factor * m[column][c];
                }
            }
        }
    }
    let mut x = [0.0_f64; 3];
    for i in 0..k {
        x[i] = m[i][3] / m[i][i];
    }
    Some(x)
}

fn station(carrier: CylinderCarrier, point: Vec3) -> f64 {
    point.sub(carrier.origin).dot(carrier.axis)
}

fn radial(carrier: CylinderCarrier, point: Vec3) -> Vec3 {
    let delta = point.sub(carrier.origin);
    delta.sub(carrier.axis.scale(delta.dot(carrier.axis)))
}

// An absent error message requests a periodic split rather than a carrier
// demotion; an ordinary construction error retains its diagnostic.
fn build_cylinder_surfaces(
    carriers: &HashMap<u32, CylinderCarrier>,
    chains: &[Chain],
    vertices: &[Vec3],
    tolerance: f64,
) -> Result<HashMap<u32, CylinderInfo>, (u32, Option<String>)> {
    let mut result = HashMap::new();
    for (&region, &carrier) in carriers {
        let relevant = chains
            .iter()
            .filter(|chain| chain.adjacent.0 == region || chain.adjacent.1 == region)
            .collect::<Vec<_>>();
        let mut low = f64::INFINITY;
        let mut high = f64::NEG_INFINITY;
        for chain in &relevant {
            for point in chain_points(chain, vertices) {
                let value = station(carrier, point);
                low = low.min(value);
                high = high.max(value);
            }
        }
        if !low.is_finite() || high - low <= tolerance {
            return Err((
                region,
                Some(format!(
                    "hybrid BREP: cylinder {region} has no finite axial span"
                )),
            ));
        }
        let mut seam = None;
        for chain in &relevant {
            if chain.closed {
                continue;
            }
            let points = chain_points(chain, vertices);
            let s0 = station(carrier, points[0]);
            let s1 = station(carrier, *points.last().unwrap());
            let r0 = radial(carrier, points[0]);
            let r1 = radial(carrier, *points.last().unwrap());
            if (s1 - s0).abs() > tolerance && r0.cross(r1).length() <= tolerance * carrier.radius {
                seam = Some(r0.normalized().map_err(|error| (region, Some(error)))?);
                break;
            }
        }
        // Without a ruling, a crossing-bore ellipse arc puts the seam at its
        // end, so the arcs that chain around the wall meet it only at vertices.
        if seam.is_none() {
            for chain in &relevant {
                let neighbor = if chain.adjacent.0 == region {
                    chain.adjacent.1
                } else {
                    chain.adjacent.0
                };
                let Some(&other) = carriers.get(&neighbor) else {
                    continue;
                };
                let points = chain_points(chain, vertices);
                if !chain.closed
                    && cylinder_pair_section_normal(carrier, other, &points, tolerance).is_some()
                {
                    seam = Some(
                        radial(carrier, points[0])
                            .normalized()
                            .map_err(|error| (region, Some(error)))?,
                    );
                    break;
                }
            }
        }
        let initial = match seam {
            Some(axis) => axis,
            None => carrier
                .axis
                .perpendicular()
                .map_err(|error| (region, Some(error)))?,
        };
        // An arbitrary frame can split an open trim across the periodic
        // seam. Choose a boundary endpoint whose meridian avoids every
        // open trim, keeping each pcurve in one surface period.
        let seam_bar = (tolerance / (TAU * carrier.radius)).max(1.0e-8);
        let candidates = std::iter::once(initial).chain(
            relevant
                .iter()
                .filter(|chain| !chain.closed)
                .flat_map(|chain| [chain.vertices[0], *chain.vertices.last().unwrap()])
                .filter_map(|vertex| radial(carrier, vertices[vertex]).normalized().ok()),
        );
        let x_axis = candidates
            .into_iter()
            .find(|&x| {
                let y = carrier.axis.cross(x);
                relevant.iter().filter(|chain| !chain.closed).all(|chain| {
                    let mut angles = chain_points(chain, vertices)
                        .iter()
                        .map(|&point| {
                            let r = radial(carrier, point);
                            r.dot(y).atan2(r.dot(x)) / TAU
                        })
                        .collect::<Vec<_>>();
                    unwrap(&mut angles);
                    let low = angles.iter().copied().fold(f64::INFINITY, f64::min);
                    let high = angles.iter().copied().fold(f64::NEG_INFINITY, f64::max);
                    let shift = (-low - seam_bar).ceil();
                    low + shift >= -seam_bar && high + shift <= 1.0 + seam_bar
                })
            })
            .ok_or((region, None))?;
        let y_axis = carrier
            .axis
            .cross(x_axis)
            .normalized()
            .map_err(|error| (region, Some(error)))?;
        let base = carrier.origin.add(carrier.axis.scale(low));
        let profile = make_line(
            base.add(x_axis.scale(carrier.radius)),
            base.add(carrier.axis.scale(high - low))
                .add(x_axis.scale(carrier.radius)),
        )
        .map_err(|error| (region, Some(error)))?;
        let surface = make_revolution(base, carrier.axis, &profile, TAU)
            .map_err(|error| (region, Some(error)))?;
        result.insert(
            region,
            CylinderInfo {
                carrier,
                base_station: low,
                height: high - low,
                x_axis,
                y_axis,
                surface,
            },
        );
    }
    Ok(result)
}

/// Cone regions that reach their apex: the region owns a mesh vertex at the
/// carrier's apex and is bounded by exactly one closed ring away from it.
/// [`HybridSwitches::pointed_cone_apex`]: a cone none of whose vertices
/// lies within its radial bar of the apex moves its apex along the axis to
/// the vertex nearest the axis, when that vertex is within the bar of it,
/// and refits the opening to its vertices by least squares (r = s tan); the
/// move is kept only when every vertex then lies within the bar of the
/// cone.
fn apex_at_vertex(
    carriers: &mut HashMap<u32, ConeCarrier>,
    triangle_regions: &[u32],
    triangles: &[[usize; 3]],
    vertices: &[Vec3],
    tolerance: f64,
) {
    for (&region, cone) in carriers.iter_mut() {
        let bar = accepted_radial_bar(tolerance * 4.0, cone.deviation);
        let own: BTreeSet<usize> = triangle_regions
            .iter()
            .zip(triangles)
            .filter(|(&owner, _)| owner == region)
            .flat_map(|(_, triangle)| triangle.iter().copied())
            .collect();
        if own.iter().any(|&vertex| vertices[vertex].sub(cone.apex).length() <= bar) {
            continue;
        }
        let Some(tip) = own
            .iter()
            .map(|&vertex| vertices[vertex])
            .min_by(|a, b| cone_radial(*cone, *a).length().total_cmp(&cone_radial(*cone, *b).length()))
        else {
            continue;
        };
        if cone_radial(*cone, tip).length() > bar {
            continue;
        }
        let apex = cone.apex.add(cone.axis.scale(cone_station(*cone, tip)));
        let moved = ConeCarrier { apex, ..*cone };
        let (rs, ss) = own.iter().fold((0.0, 0.0), |(rs, ss), &vertex| {
            let s = cone_station(moved, vertices[vertex]);
            (rs + cone_radial(moved, vertices[vertex]).length() * s, ss + s * s)
        });
        let half_angle = (rs / ss).atan();
        if !(1.0e-6..std::f64::consts::FRAC_PI_2 - 1.0e-6).contains(&half_angle) {
            continue;
        }
        let refit = ConeCarrier { half_angle, ..moved };
        let holds = own.iter().all(|&vertex| {
            let point = vertices[vertex];
            let distance = cone_radial(refit, point).length() * half_angle.cos() - cone_station(refit, point) * half_angle.sin();
            distance.abs() <= bar
        });
        if holds {
            *cone = refit;
        }
    }
}

fn pointed_cone_regions(
    carriers: &HashMap<u32, ConeCarrier>,
    triangle_regions: &[u32],
    triangles: &[[usize; 3]],
    vertices: &[Vec3],
    chains: &[Chain],
    tolerance: f64,
) -> BTreeSet<u32> {
    let mut pointed = BTreeSet::new();
    for (&region, &cone) in carriers {
        let bar = accepted_radial_bar(tolerance * 4.0, cone.deviation);
        let owns_apex = triangle_regions
            .iter()
            .zip(triangles)
            .filter(|(&owner, _)| owner == region)
            .flat_map(|(_, triangle)| triangle.iter())
            .any(|&vertex| vertices[vertex].sub(cone.apex).length() <= bar);
        if !owns_apex {
            continue;
        }
        let relevant = chains
            .iter()
            .filter(|chain| chain.adjacent.0 == region || chain.adjacent.1 == region)
            .collect::<Vec<_>>();
        let [ring] = relevant.as_slice() else {
            continue;
        };
        if !ring.closed {
            continue;
        }
        let clear_of_apex = chain_points(ring, vertices)
            .iter()
            .all(|&point| cone_station(cone, point) * cone.half_angle.tan() > bar);
        if clear_of_apex {
            pointed.insert(region);
        }
    }
    pointed
}

fn build_cone_surfaces(
    carriers: &HashMap<u32, ConeCarrier>,
    pointed_cones: &BTreeSet<u32>,
    cylinders: &HashMap<u32, CylinderInfo>,
    chains: &[Chain],
    vertices: &[Vec3],
    tolerance: f64,
    meridian_search: bool,
    search_past_cylinder: bool,
) -> Result<HashMap<u32, ConeInfo>, (u32, String)> {
    let mut result = HashMap::new();
    for (&region, &carrier) in carriers {
        let relevant = chains
            .iter()
            .filter(|chain| chain.adjacent.0 == region || chain.adjacent.1 == region)
            .collect::<Vec<_>>();
        let mut low = f64::INFINITY;
        let mut high = f64::NEG_INFINITY;
        for chain in &relevant {
            for point in chain_points(chain, vertices) {
                let value = cone_station(carrier, point);
                low = low.min(value);
                high = high.max(value);
            }
        }
        let pointed = pointed_cones.contains(&region);
        if pointed {
            // The surface starts at the apex; the ring is its only trim.
            low = 0.0;
        }
        // Any other cone reaching its apex would need a pole this builder
        // only synthesizes for a region closed by one ring.
        if !low.is_finite()
            || (!pointed && low * carrier.half_angle.tan() <= tolerance)
            || high - low <= tolerance
        {
            return Err((
                region,
                format!("hybrid BREP: cone {region} is pointed or has no finite axial span"),
            ));
        }
        // Keep fitted STL boundary samples away from the NURBS domain ends.
        // The trim loops still define the face; this small carrier extension
        // merely permits exact plane/cone intersections through quantized end
        // vertices without evaluating outside the surface domain.
        let margin = ((high - low) * 1.0e-4).max(tolerance * 2.0);
        if !pointed {
            low = (low - margin).max(tolerance / carrier.half_angle.tan());
        }
        high += margin;
        let mut seam = relevant.iter().find_map(|chain| {
            let neighbor = if chain.adjacent.0 == region {
                chain.adjacent.1
            } else {
                chain.adjacent.0
            };
            cylinders.get(&neighbor).map(|cylinder| cylinder.x_axis)
        });
        // Under `gather_cone_strips` a cylinder's seam is only the first
        // candidate (below): the cone gathered on census record
        // occt_countersunk_hole_s10_d0.005_jitter-0.3 took its bore's seam,
        // which ran through a two-point arc to a bore strip, and the whole
        // cone was demoted ("crosses the chosen seam").
        let mut from_cylinder = seam.is_some();
        for chain in &relevant {
            if chain.closed {
                continue;
            }
            let points = chain_points(chain, vertices);
            let s0 = cone_station(carrier, points[0]);
            let s1 = cone_station(carrier, *points.last().unwrap());
            let r0 = cone_radial(carrier, points[0]);
            let r1 = cone_radial(carrier, *points.last().unwrap());
            if (s1 - s0).abs() > tolerance
                && r0.cross(r1).length() <= tolerance * r0.length().max(r1.length())
            {
                seam = Some(r0.normalized().map_err(|error| (region, error))?);
                from_cylinder = false;
                break;
            }
        }
        // A pointed cone's seam runs from its ring's first vertex to the
        // apex, so the ring edge starts on it.
        if seam.is_none() && pointed {
            if let Some(ring) = relevant.first() {
                seam = cone_radial(carrier, vertices[ring.vertices[0]]).normalized().ok();
            }
        }
        let initial = match seam {
            Some(axis) => axis,
            None => carrier
                .axis
                .perpendicular()
                .map_err(|error| (region, error))?,
        };
        // [`HybridSwitches::cone_seam`]: with no ruling or cylinder seam to
        // follow, choose a meridian that avoids every open trim. Candidates
        // run midway between angularly consecutive open-chain endpoints: a
        // seam through a jittered boundary vertex starts the closed rings
        // there, off their exact circle (a 5e-3 curve/vertex gap on census
        // record occt_frustum_s1_d0.05_jitter-0.1).
        let ruled_seam = seam.is_some() && !(from_cylinder && search_past_cylinder);
        let x_axis = if !ruled_seam && !pointed && meridian_search {
            let seam_bar = (tolerance / (TAU * (high * carrier.half_angle.tan()).max(tolerance))).max(1.0e-8);
            let y0 = carrier.axis.cross(initial);
            let mut ends: Vec<f64> = relevant
                .iter()
                .filter(|chain| !chain.closed)
                .flat_map(|chain| [chain.vertices[0], *chain.vertices.last().unwrap()])
                .map(|vertex| {
                    let r = cone_radial(carrier, vertices[vertex]);
                    r.dot(y0).atan2(r.dot(initial))
                })
                .collect();
            ends.sort_by(f64::total_cmp);
            let midways = (0..ends.len()).map(|i| {
                let next = if i + 1 < ends.len() { ends[i + 1] } else { ends[0] + TAU };
                let angle = 0.5 * (ends[i] + next);
                initial.scale(angle.cos()).add(y0.scale(angle.sin()))
            });
            // Under `gather_cone_strips`, the open-chain ends too: a ring of
            // two-point arcs round the whole cone (a coarse bore's strips
            // below a gathered countersink) leaves no meridian between ends
            // that crosses none of them, but one through a shared end does.
            let at_ends: Vec<Vec3> = if search_past_cylinder {
                ends.iter().map(|&angle| initial.scale(angle.cos()).add(y0.scale(angle.sin()))).collect()
            } else {
                Vec::new()
            };
            std::iter::once(initial)
                .chain(midways)
                .chain(at_ends)
                .find(|&x| {
                    let y = carrier.axis.cross(x);
                    relevant.iter().filter(|chain| !chain.closed).all(|chain| {
                        let mut angles = chain_points(chain, vertices)
                            .iter()
                            .map(|&point| {
                                let r = cone_radial(carrier, point);
                                r.dot(y).atan2(r.dot(x)) / TAU
                            })
                            .collect::<Vec<_>>();
                        unwrap(&mut angles);
                        let low = angles.iter().copied().fold(f64::INFINITY, f64::min);
                        let high = angles.iter().copied().fold(f64::NEG_INFINITY, f64::max);
                        let shift = (-low - seam_bar).ceil();
                        low + shift >= -seam_bar && high + shift <= 1.0 + seam_bar
                    })
                })
                .unwrap_or(initial)
        } else {
            initial
        };
        let y_axis = carrier
            .axis
            .cross(x_axis)
            .normalized()
            .map_err(|error| (region, error))?;
        let radius_low = low * carrier.half_angle.tan();
        let radius_high = high * carrier.half_angle.tan();
        let base = carrier.apex.add(carrier.axis.scale(low));
        let profile = make_line(
            base.add(x_axis.scale(radius_low)),
            base.add(carrier.axis.scale(high - low))
                .add(x_axis.scale(radius_high)),
        )
        .map_err(|error| (region, error))?;
        let surface =
            make_revolution(base, carrier.axis, &profile, TAU).map_err(|error| (region, error))?;
        result.insert(
            region,
            ConeInfo {
                carrier,
                pointed,
                base_station: low,
                height: high - low,
                x_axis,
                y_axis,
                surface,
            },
        );
    }
    Ok(result)
}

/// Frame each sphere as a revolution about the common normal of the ring
/// planes that bound it, pointing into the region so a single-ring cap
/// holds the north pole; the seam passes through the first ring's first
/// source vertex.
///
/// A sphere whose rings are NOT all coaxial (a sphere clipped by a box: six
/// flats on three axes) is framed about one ring axis whose two poles both
/// fall in removed caps of coaxial rings, with its seam meridian turned clear
/// of every other ring. Those tilted rings are exact plane/sphere circles
/// whose sphere pcurves are fitted, not isoparametric; see
/// [`tilted_sphere_ring_edge`].
#[allow(clippy::too_many_arguments)]
fn build_sphere_surfaces(
    carriers: &HashMap<u32, SphereCarrier>,
    planes: &HashMap<u32, PlaneInfo>,
    cylinders: &HashMap<u32, CylinderInfo>,
    cones: &HashMap<u32, ConeInfo>,
    chains: &[Chain],
    vertices: &[Vec3],
    region_centroids: &HashMap<u32, Vec3>,
    region_samples: &HashMap<u32, Vec<Vec3>>,
    tolerance: f64,
) -> Result<HashMap<u32, SphereInfo>, (u32, String)> {
    let mut result = HashMap::new();
    for (&region, &carrier) in carriers {
        let relevant = chains
            .iter()
            .filter(|chain| chain.adjacent.0 == region || chain.adjacent.1 == region)
            .collect::<Vec<_>>();
        if relevant.is_empty() {
            return Err((
                region,
                format!("hybrid BREP: sphere {region} is not bounded by closed rings"),
            ));
        }
        if relevant.iter().any(|chain| !chain.closed) {
            let samples = region_samples.get(&region).map(Vec::as_slice).unwrap_or(&[]);
            let info = sphere_patch_info(
                region, carrier, &relevant, cylinders, cones, vertices, samples, tolerance,
            )
            .map_err(|message| (region, message))?;
            result.insert(region, info);
            continue;
        }
        let mut axis: Option<Vec3> = None;
        // A ring shared with a coaxial revolve is one edge whose pcurve on
        // each face starts at that face's seam, so the seams must coincide.
        let mut seam: Option<Vec3> = None;
        let angular = tolerance / carrier.radius;
        let mut rings = Vec::with_capacity(relevant.len());
        let mut coaxial = true;
        for chain in &relevant {
            let neighbor = if chain.adjacent.0 == region {
                chain.adjacent.1
            } else {
                chain.adjacent.0
            };
            let (direction, seam_hint) = if let Some(plane) = planes.get(&neighbor) {
                (plane.normal.normalized().map_err(|error| (region, error))?, None)
            } else if let Some(cylinder) = cylinders.get(&neighbor) {
                (cylinder.carrier.axis, Some(cylinder.x_axis))
            } else if let Some(cone) = cones.get(&neighbor) {
                (cone.carrier.axis, Some(cone.x_axis))
            } else {
                return Err((
                    region,
                    format!("hybrid BREP: sphere {region} ring neighbor is not a plane or coaxial revolve"),
                ));
            };
            let points = chain_points(chain, vertices);
            let station = points
                .iter()
                .map(|point| point.sub(carrier.center).dot(direction))
                .sum::<f64>()
                / points.len() as f64;
            rings.push(SphereRing {
                direction,
                station,
                plane: planes.contains_key(&neighbor),
                seam_hint,
                first_point: points[0],
            });
            match axis {
                None => axis = Some(direction),
                Some(current) if current.cross(direction).length() <= angular => {}
                Some(_) => coaxial = false,
            }
        }
        let (mut axis, seam, north_pole_inside) = if coaxial {
            for ring in &rings {
                if let Some(hint) = ring.seam_hint {
                    seam.get_or_insert(hint);
                }
            }
            (axis.unwrap(), seam, relevant.len() == 1)
        } else {
            let samples = region_samples.get(&region).map(Vec::as_slice).unwrap_or(&[]);
            let (axis, seam) = tilted_sphere_frame(carrier, &mut rings, samples, tolerance)
                .map_err(|message| {
                    (
                        region,
                        format!("hybrid BREP: sphere {region} rings are not coaxial and {message}"),
                    )
                })?;
            (axis, Some(seam), false)
        };
        // Point the axis into the region: a cap then surrounds the north
        // pole. A tilted frame has both poles in removed caps, so either
        // sense serves and the one chosen stays.
        if coaxial {
            if let Some(&centroid) = region_centroids.get(&region) {
                if centroid.sub(carrier.center).dot(axis) < 0.0 {
                    axis = axis.scale(-1.0);
                }
            }
        }
        let seam = seam.unwrap_or_else(|| rings[0].first_point.sub(carrier.center));
        let surface = make_sphere_surface_framed(carrier.center, carrier.radius, axis, Some(seam))
            .map_err(|error| (region, error))?;
        let equator = surface.evaluate(0.0, 0.5).map_err(|error| (region, error))?;
        let x_axis = equator
            .sub(carrier.center)
            .normalized()
            .map_err(|error| (region, error))?;
        let y_axis = axis.cross(x_axis).normalized().map_err(|error| (region, error))?;
        result.insert(
            region,
            SphereInfo {
                carrier,
                axis,
                x_axis,
                y_axis,
                surface,
                north_pole_inside,
            },
        );
    }
    Ok(result)
}

/// A sphere patch bounded by open arcs, such as the corner octant of a box
/// rounded on every edge: one loop of three quarter-arcs, each shared with an
/// edge blend on a different axis, so no polar axis makes them latitudes.
///
/// Every arc must be shared with a coaxial cylinder or cone, whose arc is
/// the edge's curve; the sphere side is a fitted pcurve (see
/// [`sphere_patch_pcurve`]). The frame keeps the whole patch in the open
/// hemisphere around its mean direction `m`: the polar axis is square to `m`
/// and the seam meridian passes through `-m`, so both poles and the seam are
/// at least `90° - (widest angle from m)` away from every point of the patch,
/// and that clearance must exceed the tilted lane's bar.
#[allow(clippy::too_many_arguments)]
fn sphere_patch_info(
    region: u32,
    carrier: SphereCarrier,
    relevant: &[&Chain],
    cylinders: &HashMap<u32, CylinderInfo>,
    cones: &HashMap<u32, ConeInfo>,
    vertices: &[Vec3],
    samples: &[Vec3],
    tolerance: f64,
) -> Result<SphereInfo, String> {
    for chain in relevant {
        let neighbor = if chain.adjacent.0 == region {
            chain.adjacent.1
        } else {
            chain.adjacent.0
        };
        if !cylinders.contains_key(&neighbor) && !cones.contains_key(&neighbor) {
            return Err(format!(
                "hybrid BREP: sphere {region} is bounded by open arcs and one is not shared with a cylinder or cone"
            ));
        }
    }
    let directions = relevant
        .iter()
        .flat_map(|chain| chain_points(chain, vertices))
        .chain(samples.iter().copied())
        .map(|point| point.sub(carrier.center).normalized())
        .collect::<Result<Vec<_>, String>>()?;
    let mean = directions
        .iter()
        .fold(Vec3::new(0.0, 0.0, 0.0), |sum, direction| sum.add(*direction))
        .normalized()
        .map_err(|_| format!("hybrid BREP: sphere {region} patch has no mean direction"))?;
    let widest = directions
        .iter()
        .map(|direction| direction.dot(mean).clamp(-1.0, 1.0).acos())
        .fold(0.0, f64::max);
    let clearance = std::f64::consts::FRAC_PI_2 - widest;
    if clearance * carrier.radius <= 16.0 * tolerance {
        return Err(format!(
            "hybrid BREP: sphere {region} patch does not fit in a hemisphere clear of a pole and seam (widest {widest:.4} rad)"
        ));
    }
    let axis = mean.perpendicular()?;
    let seam = mean.scale(-1.0);
    let surface = make_sphere_surface_framed(carrier.center, carrier.radius, axis, Some(seam))?;
    let equator = surface.evaluate(0.0, 0.5)?;
    let x_axis = equator.sub(carrier.center).normalized()?;
    let y_axis = axis.cross(x_axis).normalized()?;
    Ok(SphereInfo {
        carrier,
        axis,
        x_axis,
        y_axis,
        surface,
        north_pole_inside: false,
    })
}

/// The pcurve on a patch sphere (see [`sphere_patch_info`]) of an open arc
/// whose curve the neighbouring revolve built, oriented along the curve and
/// checked against it at the exact-topology bar. The patch frame keeps the
/// seam clear of the arc, so the pcurve never wraps.
fn sphere_patch_pcurve(
    chain_index: usize,
    sphere: &SphereInfo,
    curve: &NurbsCurve,
    t0: f64,
    t1: f64,
    tolerance: f64,
) -> Result<NurbsCurve, String> {
    let pcurve = build_pcurve_on_surface_range(
        &sphere.surface,
        curve,
        t0,
        t1,
        true,
        (tolerance * 0.05).max(1.0e-9),
    )?;
    let domain = pcurve.domain()?;
    let mut previous_u: Option<f64> = None;
    for sample in 0..=64 {
        let fraction = sample as f64 / 64.0;
        let t = t0 + (t1 - t0) * fraction;
        let q = domain[0] + (domain[1] - domain[0]) * fraction;
        let uv = pcurve.evaluate(q)?;
        let surface_point = sphere.surface.evaluate(uv.x, uv.y)?;
        if curve.evaluate(t)?.sub(surface_point).length() > tolerance * 0.1
            || previous_u.is_some_and(|u| (uv.x - u).abs() > 0.25)
        {
            return Err(format!(
                "hybrid BREP: sphere patch arc {chain_index} pcurve failed agreement"
            ));
        }
        previous_u = Some(uv.x);
    }
    Ok(pcurve)
}

/// One closed ring bounding a sphere region: the plane `(p - center) ·
/// direction = station` it lies in, and what the neighbor across it demands.
struct SphereRing {
    direction: Vec3,
    station: f64,
    plane: bool,
    seam_hint: Option<Vec3>,
    first_point: Vec3,
}

/// Polar axis and seam direction for a sphere region bounded by rings on
/// several axes, or why there is none.
///
/// The region must be the sphere less DISJOINT caps, one per ring: every
/// sample of the region lies on the kept side of every ring. The polar axis
/// is a ring axis whose two poles both lie strictly inside removed caps of
/// rings coaxial with it, so those rings stay latitudes and no pole touches
/// the face. Every other ring must be a plane circle, and the seam meridian
/// is turned to clear each such ring's removed cap by a margin, so its
/// fitted pcurve is a closed loop inside the parameter square.
fn tilted_sphere_frame(
    carrier: SphereCarrier,
    rings: &mut [SphereRing],
    samples: &[Vec3],
    tolerance: f64,
) -> Result<(Vec3, Vec3), String> {
    let radius = carrier.radius;
    let angular = tolerance / radius;
    // Orient each ring so `(p - center) · direction >= station` is the KEPT
    // side: the removed cap is then centred on `-direction`.
    for ring in rings.iter_mut() {
        let side = samples
            .iter()
            .map(|sample| sample.sub(carrier.center).dot(ring.direction) - ring.station)
            .max_by(|a, b| a.abs().total_cmp(&b.abs()))
            .ok_or_else(|| "the region has no samples to side its rings".to_owned())?;
        if side < 0.0 {
            ring.direction = ring.direction.scale(-1.0);
            ring.station = -ring.station;
        }
        if ring.station.abs() >= radius - tolerance {
            return Err("a ring is degenerate".to_owned());
        }
    }
    let slack = tolerance * 4.0;
    if samples.iter().any(|sample| {
        rings
            .iter()
            .any(|ring| sample.sub(carrier.center).dot(ring.direction) < ring.station - slack)
    }) {
        return Err("the region is not the sphere less one cap per ring".to_owned());
    }
    // Removed caps must be disjoint: the angle between two cap centres must
    // exceed the sum of their angular radii. A cap centred on `-direction`
    // holds the points with `(p - center) · -direction > -station`.
    let cap_radius = |ring: &SphereRing| (-ring.station / radius).clamp(-1.0, 1.0).acos();
    for (index, first) in rings.iter().enumerate() {
        for second in &rings[index + 1..] {
            let between = first.direction.dot(second.direction).clamp(-1.0, 1.0).acos();
            if between <= cap_radius(first) + cap_radius(second) + angular {
                return Err("two removed caps touch".to_owned());
            }
        }
    }
    // Strictly inside a removed cap, clear of its rim.
    let in_removed_cap = |ring: &SphereRing, pole: Vec3| {
        pole.dot(ring.direction) * radius < ring.station - slack
    };
    for candidate in rings.iter() {
        let axis = candidate.direction;
        let parallel = |ring: &SphereRing| axis.cross(ring.direction).length() <= angular;
        let north_capped = rings
            .iter()
            .any(|ring| parallel(ring) && in_removed_cap(ring, axis));
        let south_capped = rings
            .iter()
            .any(|ring| parallel(ring) && in_removed_cap(ring, axis.scale(-1.0)));
        if !north_capped || !south_capped {
            continue;
        }
        let tilted = rings.iter().filter(|ring| !parallel(ring)).collect::<Vec<_>>();
        if tilted.iter().any(|ring| !ring.plane) {
            continue;
        }
        let Ok(x0) = axis.perpendicular() else {
            continue;
        };
        let Ok(y0) = axis.cross(x0).normalized() else {
            continue;
        };
        // How far the meridian at azimuth `phi` stays outside every tilted
        // cap, as an angle. The meridian's closest approach to a direction
        // `w` has cosine `max(hypot(a, b), |b|)` for `a` the component of
        // `w` along the meridian's equator direction and `b` along the axis.
        let clearance = |phi: f64| {
            let toward = x0.scale(phi.cos()).add(y0.scale(phi.sin()));
            tilted
                .iter()
                .map(|ring| {
                    let cap = ring.direction.scale(-1.0);
                    let a = cap.dot(toward);
                    let b = cap.dot(axis);
                    let closest = if a >= 0.0 { a.hypot(b) } else { b.abs() };
                    closest.clamp(-1.0, 1.0).acos() - cap_radius(ring)
                })
                .fold(f64::INFINITY, f64::min)
        };
        let forced = rings
            .iter()
            .filter(|ring| parallel(ring))
            .find_map(|ring| ring.seam_hint);
        let phi = match forced {
            Some(hint) => hint.dot(y0).atan2(hint.dot(x0)),
            None => (0..720)
                .map(|step| step as f64 * TAU / 720.0)
                .max_by(|a, b| clearance(*a).total_cmp(&clearance(*b)))
                .unwrap(),
        };
        if clearance(phi) * radius <= 16.0 * tolerance {
            continue;
        }
        return Ok((axis, x0.scale(phi.cos()).add(y0.scale(phi.sin()))));
    }
    Err("no ring axis has both poles in removed coaxial caps with a seam clear of the other rings".to_owned())
}

fn sphere_station(info: &SphereInfo, point: Vec3) -> f64 {
    point.sub(info.carrier.center).dot(info.axis)
}

fn sphere_coordinates(info: &SphereInfo, point: Vec3) -> [f64; 2] {
    let d = point.sub(info.carrier.center);
    let angle = d.dot(info.y_axis).atan2(d.dot(info.x_axis)).rem_euclid(TAU);
    let u = circle_angle_to_parameter(4, TAU, angle);
    // The meridian is a half circle from the south pole: two rational
    // quadratic spans over the polar angle.
    let polar = (sphere_station(info, point) / info.carrier.radius)
        .clamp(-1.0, 1.0)
        .asin();
    let v = circle_angle_to_parameter(2, std::f64::consts::PI, polar + std::f64::consts::FRAC_PI_2);
    [u, v]
}

fn max_line_deviation(points: &[Vec3]) -> f64 {
    if points.len() < 3 {
        return 0.0;
    }
    let start = points[0];
    let chord = points[points.len() - 1].sub(start);
    let denominator = chord.dot(chord);
    points[1..points.len() - 1]
        .iter()
        .map(|point| {
            let delta = point.sub(start);
            let t = if denominator > 0.0 {
                (delta.dot(chord) / denominator).clamp(0.0, 1.0)
            } else {
                0.0
            };
            delta.sub(chord.scale(t)).length()
        })
        .fold(0.0, f64::max)
}

fn trim_curve(curve: &NurbsCurve, start: f64, end: f64) -> Result<NurbsCurve, String> {
    let domain = curve.domain()?;
    // `split` refuses a parameter within the kernel's knot identity
    // tolerance of either end, so a trim that close to an end is the whole
    // curve: a station a fraction of a nanometre above a region's own
    // minimum (the fitted axis is not exactly a coordinate direction, so
    // two vertices on one rim differ in the last bits) is that region's
    // base, not a cut.
    let mut trimmed = if end < domain[1] - KNOT_IDENTITY_TOL {
        curve.split(end)?.0
    } else {
        curve.clone()
    };
    if start > domain[0] + KNOT_IDENTITY_TOL {
        trimmed = trimmed.split(start)?.1;
    }
    Ok(trimmed)
}

fn cylinder_coordinates(info: &CylinderInfo, point: Vec3) -> [f64; 2] {
    let r = radial(info.carrier, point);
    let angle = r.dot(info.y_axis).atan2(r.dot(info.x_axis)).rem_euclid(TAU);
    // A rational quadratic circle is not angle-linear inside a knot span.
    // Use the public exact angle-to-NURBS mapping used by the kernel carrier.
    let u = circle_angle_to_parameter(4, TAU, angle);
    let v = (station(info.carrier, point) - info.base_station) / info.height;
    [u, v]
}

fn cone_coordinates(info: &ConeInfo, point: Vec3) -> [f64; 2] {
    let r = cone_radial(info.carrier, point);
    let angle = r.dot(info.y_axis).atan2(r.dot(info.x_axis)).rem_euclid(TAU);
    let u = circle_angle_to_parameter(4, TAU, angle);
    let v = (cone_station(info.carrier, point) - info.base_station) / info.height;
    [u, v]
}

/// The exact circle where a plane cuts a sphere, as the edge of a closed
/// chain that is NOT a latitude of the sphere's frame, with its fitted pcurve
/// on the sphere (oriented along the curve).
///
/// The circle starts at the source ring's first vertex, so the edge's one
/// vertex is where the chain begins. The pcurve is checked against the
/// circle at the exact-topology bar; the frame's seam was turned clear of
/// this ring, so the pcurve is a closed loop that never wraps.
fn tilted_sphere_ring_edge(
    chain_index: usize,
    sphere: &SphereInfo,
    plane: PlaneInfo,
    points: &[Vec3],
    tolerance: f64,
) -> Result<(NurbsCurve, f64, f64, bool, NurbsCurve), String> {
    let normal = plane.normal.normalized()?;
    let center = sphere.carrier.center;
    let offset = plane.origin.sub(center).dot(normal);
    let radius_sq = sphere.carrier.radius * sphere.carrier.radius - offset * offset;
    if radius_sq <= tolerance * tolerance {
        return Err(format!(
            "hybrid BREP: tilted sphere ring {chain_index} misses its sphere"
        ));
    }
    let ring_radius = radius_sq.sqrt();
    let ring_center = center.add(normal.scale(offset));
    let radial_bar = accepted_radial_bar(tolerance, sphere.carrier.deviation);
    for &point in points {
        let delta = point.sub(ring_center);
        let height = delta.dot(normal);
        let in_plane = delta.sub(normal.scale(height)).length();
        if height.abs() > tolerance || (in_plane - ring_radius).abs() > radial_bar {
            return Err(format!(
                "hybrid BREP: tilted sphere ring {chain_index} source leaves its circle (height {height:.3e}, radial {:.3e}, bar {radial_bar:.3e})",
                in_plane - ring_radius
            ));
        }
    }
    let first = points[0].sub(ring_center);
    let x = first.sub(normal.scale(first.dot(normal))).normalized()?;
    let y = normal.cross(x).normalized()?;
    // The chain winds one way about the normal; the circle runs the same way
    // when its second point turns positively.
    let angles = points
        .iter()
        .chain(std::iter::once(&points[0]))
        .map(|point| {
            let delta = point.sub(ring_center);
            delta.dot(y).atan2(delta.dot(x)) / TAU
        })
        .collect::<Vec<_>>();
    let mut unwrapped = angles.clone();
    unwrap(&mut unwrapped);
    let sweep = unwrapped.last().unwrap() - unwrapped[0];
    if (sweep.abs() - 1.0).abs() > 0.02 || !monotone_parameter_walk(&unwrapped, 1.0e-5) {
        return Err(format!(
            "hybrid BREP: tilted sphere ring {chain_index} does not wind once (sweep {sweep:.6})"
        ));
    }
    let curve = make_arc(ring_center, x, y, ring_radius, 0.0, TAU)?;
    let domain = curve.domain()?;
    let pcurve = build_pcurve_on_surface_range(
        &sphere.surface,
        &curve,
        domain[0],
        domain[1],
        true,
        (tolerance * 0.05).max(1.0e-9),
    )?;
    let pcurve_domain = pcurve.domain()?;
    let mut previous_u: Option<f64> = None;
    for sample in 0..=64 {
        let fraction = sample as f64 / 64.0;
        let t = domain[0] + (domain[1] - domain[0]) * fraction;
        let q = pcurve_domain[0] + (pcurve_domain[1] - pcurve_domain[0]) * fraction;
        let uv = pcurve.evaluate(q)?;
        let surface_point = sphere.surface.evaluate(uv.x, uv.y)?;
        if curve.evaluate(t)?.sub(surface_point).length() > tolerance * 0.1
            || previous_u.is_some_and(|u| (uv.x - u).abs() > 0.25)
        {
            return Err(format!(
                "hybrid BREP: tilted sphere ring {chain_index} pcurve failed agreement"
            ));
        }
        previous_u = Some(uv.x);
    }
    let start = pcurve.evaluate(pcurve_domain[0])?;
    let end = pcurve.evaluate(pcurve_domain[1])?;
    if (start.x - end.x).abs() > 1.0e-9 || (start.y - end.y).abs() > 1.0e-9 {
        return Err(format!(
            "hybrid BREP: tilted sphere ring {chain_index} pcurve does not close"
        ));
    }
    Ok((curve, domain[0], domain[1], sweep > 0.0, pcurve))
}

/// The exact ellipse arc two crossing cylinders of one radius share, with
/// its fitted pcurve on each (oriented along the curve).
///
/// In its section plane through the axes' crossing point the ellipse has
/// the common radius as its semi-minor axis, along the common normal of the
/// axes, and `r / sqrt(1 - (e · axis)^2)` along the in-plane direction `e`
/// normal to that, the same for both walls. Both pcurves are checked against
/// the arc, and each must stay inside its wall's parameter square without
/// wrapping: the wall's seam was put at the end of one of these arcs.
#[allow(clippy::type_complexity)]
fn cylinder_pair_section_edge(
    chain_index: usize,
    walls: [(u32, &CylinderInfo); 2],
    points: &[Vec3],
    tolerance: f64,
) -> Result<(NurbsCurve, f64, f64, bool, Vec<(u32, NurbsCurve)>), String> {
    let [(_, first), (_, second)] = walls;
    let (center, normal) =
        cylinder_pair_section_normal(first.carrier, second.carrier, points, tolerance)
            .ok_or_else(|| {
                format!(
                    "hybrid BREP: cylinder/cylinder chain {chain_index} is not an equal-radius crossing section"
                )
            })?;
    let radius = 0.5 * (first.carrier.radius + second.carrier.radius);
    let minor = first.carrier.axis.cross(second.carrier.axis).normalized()?;
    let major = normal.cross(minor).normalized()?;
    let along = major.dot(first.carrier.axis.normalized()?);
    let major_radius = radius / (1.0 - along * along).sqrt();
    let mut angles = points
        .iter()
        .map(|point| {
            let delta = point.sub(center);
            (delta.dot(minor) / radius).atan2(delta.dot(major) / major_radius)
        })
        .collect::<Vec<_>>();
    for index in 1..angles.len() {
        while angles[index] - angles[index - 1] > std::f64::consts::PI {
            angles[index] -= TAU;
        }
        while angles[index] - angles[index - 1] < -std::f64::consts::PI {
            angles[index] += TAU;
        }
    }
    if !monotone_parameter_walk(&angles, 1.0e-5) {
        return Err(format!(
            "hybrid BREP: cylinder/cylinder chain {chain_index} backtracks along its ellipse"
        ));
    }
    let (a0, a1) = (angles[0], *angles.last().unwrap());
    let (low, high) = (a0.min(a1), a0.max(a1));
    if high - low >= TAU - 1.0e-6 {
        return Err(format!(
            "hybrid BREP: cylinder/cylinder chain {chain_index} spans its whole ellipse"
        ));
    }
    // A unit arc in the xy plane, mapped affinely onto the ellipse: exact.
    let unit = make_arc(
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        1.0,
        low,
        high,
    )?;
    let controls = unit
        .control_points
        .iter()
        .map(|control| {
            let point = center
                .add(major.scale(major_radius * control.x / control.w))
                .add(minor.scale(radius * control.y / control.w));
            Vec4::from_point(point, control.w)
        })
        .collect::<Vec<_>>();
    let curve = NurbsCurve::new(unit.degree, unit.knots.clone(), controls)?;
    let domain = curve.domain()?;
    let bar = accepted_radial_bar(
        tolerance,
        first.carrier.deviation.max(second.carrier.deviation),
    );
    for &point in points {
        let distance = project_point_to_curve(&curve, point)?.distance;
        if distance > bar {
            return Err(format!(
                "hybrid BREP: cylinder/cylinder chain {chain_index} source leaves its ellipse by {distance:.3e} (bar {bar:.3e})"
            ));
        }
    }
    let mut pcurves = Vec::with_capacity(2);
    for (owner, wall) in walls {
        let pcurve = build_pcurve_on_surface_range(
            &wall.surface,
            &curve,
            domain[0],
            domain[1],
            true,
            (tolerance * 0.05).max(1.0e-9),
        )?;
        let pcurve_domain = pcurve.domain()?;
        let mut previous_u: Option<f64> = None;
        for sample in 0..=32 {
            let fraction = sample as f64 / 32.0;
            let t = domain[0] + (domain[1] - domain[0]) * fraction;
            let q = pcurve_domain[0] + (pcurve_domain[1] - pcurve_domain[0]) * fraction;
            let uv = pcurve.evaluate(q)?;
            let surface_point = wall.surface.evaluate(uv.x, uv.y)?;
            let miss = curve.evaluate(t)?.sub(surface_point).length();
            if miss > bar || previous_u.is_some_and(|u| (uv.x - u).abs() > 0.25) {
                return Err(format!(
                    "hybrid BREP: cylinder/cylinder chain {chain_index} pcurve on region {owner} failed agreement (miss {miss:.3e}, bar {bar:.3e})"
                ));
            }
            previous_u = Some(uv.x);
        }
        pcurves.push((owner, pcurve));
    }
    Ok((curve, domain[0], domain[1], a1 > a0, pcurves))
}

fn exact_local_cone_plane_conic(
    info: &ConeInfo,
    plane: PlaneInfo,
    points: &[Vec3],
    tolerance: f64,
) -> Result<NurbsCurve, String> {
    let mut angles = points
        .iter()
        .map(|&point| {
            let radial = cone_radial(info.carrier, point);
            radial.dot(info.y_axis).atan2(radial.dot(info.x_axis))
        })
        .collect::<Vec<_>>();
    for index in 1..angles.len() {
        while angles[index] - angles[index - 1] > std::f64::consts::PI {
            angles[index] -= TAU;
        }
        while angles[index] - angles[index - 1] < -std::f64::consts::PI {
            angles[index] += TAU;
        }
    }
    if !monotone_parameter_walk(&angles, 1.0e-5) {
        return Err("hybrid BREP: cone/plane source angles are not monotone".into());
    }
    let low = angles[0].min(*angles.last().unwrap());
    let high = angles[0].max(*angles.last().unwrap());
    if high - low <= 1.0e-10 || high - low > TAU + 1.0e-10 {
        return Err("hybrid BREP: cone/plane source angle span is invalid".into());
    }

    // Map a finite exact reference-circle arc projectively through the cone
    // apex onto the cutting plane. Unlike mapping a full circle, this keeps
    // hyperbolic/parabolic branches one-signed over the actual trim interval.
    let reference_station = info.base_station + 0.5 * info.height;
    let reference_radius = reference_station * info.carrier.half_angle.tan();
    let center = info
        .carrier
        .apex
        .add(info.carrier.axis.scale(reference_station));
    let circle = make_arc(
        center,
        info.x_axis,
        info.y_axis,
        reference_radius,
        low,
        high,
    )?;
    let normal = plane.normal.normalized()?;
    let apex = info.carrier.apex;
    let scale = plane.origin.sub(apex).dot(normal);
    if scale.abs() <= tolerance {
        return Err("hybrid BREP: cone/plane section passes through the apex".into());
    }
    let mut controls = circle
        .control_points
        .iter()
        .map(|point| {
            let relative = Vec3::new(
                point.x - point.w * apex.x,
                point.y - point.w * apex.y,
                point.z - point.w * apex.z,
            );
            let weight = relative.dot(normal);
            let scaled = relative.scale(scale);
            Vec4 {
                x: apex.x * weight + scaled.x,
                y: apex.y * weight + scaled.y,
                z: apex.z * weight + scaled.z,
                w: weight,
            }
        })
        .collect::<Vec<_>>();
    if controls.iter().any(|point| point.w.abs() <= 1.0e-12) {
        return Err("hybrid BREP: cone/plane local conic crosses an asymptote".into());
    }
    let sign = controls[0].w.signum();
    if controls.iter().any(|point| point.w.signum() != sign) {
        return Err("hybrid BREP: cone/plane local conic has mixed weight signs".into());
    }
    if sign < 0.0 {
        for point in &mut controls {
            point.x = -point.x;
            point.y = -point.y;
            point.z = -point.z;
            point.w = -point.w;
        }
    }
    NurbsCurve::new(circle.degree, circle.knots, controls)
}

fn unwrap(values: &mut [f64]) {
    for index in 1..values.len() {
        while values[index] - values[index - 1] > 0.5 {
            values[index] -= 1.0;
        }
        while values[index] - values[index - 1] < -0.5 {
            values[index] += 1.0;
        }
    }
}

fn monotone_parameter_walk(values: &[f64], tolerance: f64) -> bool {
    let net = values.last().unwrap() - values[0];
    if net.abs() <= tolerance {
        return false;
    }
    let sign = net.signum();
    let variation = values
        .windows(2)
        .map(|pair| pair[1] - pair[0])
        .try_fold(0.0, |sum, step| {
            (step * sign >= -tolerance).then_some(sum + step.abs())
        });
    variation.is_some_and(|total| (total - net.abs()).abs() <= tolerance * values.len() as f64)
}

/// A boolean cut through a polygonal cylinder puts its ruling on a chord,
/// not on the circle. Radially projecting each arc end then leaves the
/// adjoining plane/plane edge behind. Reconstruct the ruling on BOTH its
/// carriers first, so every chain uses the same corrected corner.
fn snap_cylinder_ruling_vertices(
    vertices: &mut [Vec3],
    chains: &[Chain],
    planes: &HashMap<u32, PlaneInfo>,
    cylinders: &HashMap<u32, CylinderCarrier>,
    tolerance: f64,
) {
    let mut incident = HashMap::<usize, BTreeSet<u32>>::new();
    for chain in chains {
        for &vertex in &chain.vertices {
            incident
                .entry(vertex)
                .or_default()
                .extend([chain.adjacent.0, chain.adjacent.1]);
        }
    }
    let source = vertices.to_vec();
    let mut proposals = BTreeMap::<usize, Vec<Vec3>>::new();
    for chain in chains.iter().filter(|chain| !chain.closed) {
        let (cylinder, plane) = match (
            cylinders.get(&chain.adjacent.0),
            planes.get(&chain.adjacent.1),
            cylinders.get(&chain.adjacent.1),
            planes.get(&chain.adjacent.0),
        ) {
            (Some(cylinder), Some(plane), _, _) | (_, _, Some(cylinder), Some(plane)) => {
                (cylinder, plane)
            }
            _ => continue,
        };
        let points = chain_points(chain, &source);
        let span = points.last().unwrap().sub(points[0]);
        // Only axial rulings; rings and oblique sections keep their own lane.
        if span.dot(cylinder.axis).abs() <= tolerance
            || span.cross(cylinder.axis).length() > tolerance
            || max_line_deviation(&points) > tolerance
        {
            continue;
        }
        let normal = plane.normal;
        let radial_normal = normal.sub(cylinder.axis.scale(normal.dot(cylinder.axis)));
        let Ok(x) = radial_normal.normalized() else {
            continue;
        };
        if normal.dot(cylinder.axis).abs() * span.length() > tolerance {
            continue;
        }
        let y = cylinder.axis.cross(x);
        let bar = accepted_radial_bar(tolerance, cylinder.deviation);
        for (&vertex, point) in chain.vertices.iter().zip(points) {
            let center = cylinder.origin.add(
                cylinder
                    .axis
                    .scale(point.sub(cylinder.origin).dot(cylinder.axis)),
            );
            let offset = plane.origin.sub(center).dot(normal) / radial_normal.length();
            let height_squared = cylinder.radius * cylinder.radius - offset * offset;
            if height_squared <= 0.0 {
                continue;
            }
            let height = height_squared.sqrt();
            let base = center.add(x.scale(offset));
            let a = base.add(y.scale(height));
            let b = base.sub(y.scale(height));
            let exact = if a.sub(point).length() < b.sub(point).length() {
                a
            } else {
                b
            };
            if !exact.sub(point).length().is_finite() || exact.sub(point).length() > bar {
                continue;
            }
            // Do not move a corner off any other incident carrier, including
            // a facet produced by a previous local demotion. Unsupported
            // neighbors keep their original vertices.
            let compatible = incident[&vertex].iter().all(|region| {
                if let Some(plane) = planes.get(region) {
                    exact.sub(plane.origin).dot(plane.normal).abs() <= tolerance
                } else if let Some(cylinder) = cylinders.get(region) {
                    let delta = exact.sub(cylinder.origin);
                    (delta
                        .sub(cylinder.axis.scale(delta.dot(cylinder.axis)))
                        .length()
                        - cylinder.radius)
                        .abs()
                        <= tolerance
                } else {
                    false
                }
            });
            if compatible {
                proposals.entry(vertex).or_default().push(exact);
            }
        }
    }
    for (vertex, candidates) in proposals {
        if candidates
            .iter()
            .all(|p| p.sub(candidates[0]).length() <= tolerance)
        {
            vertices[vertex] = candidates[0];
        }
    }
}

fn exact_arena_vertex(
    mesh_vertex: usize,
    exact_point: Vec3,
    tolerance: f64,
    arena: &mut TopologyArena,
    arena_vertices: &mut HashMap<usize, brep_kernel::VertexId>,
    ids: &mut Ids,
) -> Result<brep_kernel::VertexId, String> {
    if let Some(&vertex) = arena_vertices.get(&mesh_vertex) {
        let existing = arena
            .vertices
            .get(vertex)
            .ok_or_else(|| "hybrid BREP: reused arena vertex disappeared".to_owned())?;
        let disagreement = existing.point.sub(exact_point).length();
        if !disagreement.is_finite() || disagreement > tolerance {
            return Err(format!(
                "hybrid BREP: exact curves disagree by {disagreement:.6e} (bar {tolerance:.6e}) at shared mesh vertex {mesh_vertex}: ({:.6},{:.6},{:.6}) vs ({:.6},{:.6},{:.6})",
                existing.point.x,
                existing.point.y,
                existing.point.z,
                exact_point.x,
                exact_point.y,
                exact_point.z,
            ));
        }
        return Ok(vertex);
    }
    let vertex = arena.vertices.insert(ArenaVertex {
        wire_id: ids.next(),
        point: exact_point,
    });
    arena_vertices.insert(mesh_vertex, vertex);
    Ok(vertex)
}

#[derive(Clone, Copy)]
enum RevolveInfoRef<'a> {
    Cylinder(&'a CylinderInfo),
    Cone(&'a ConeInfo),
    Sphere(&'a SphereInfo),
    Torus(&'a TorusInfo),
}

impl<'a> RevolveInfoRef<'a> {
    fn surface(self) -> &'a NurbsSurface {
        match self {
            Self::Cylinder(info) => &info.surface,
            Self::Cone(info) => &info.surface,
            Self::Sphere(info) => &info.surface,
            Self::Torus(info) => &info.surface,
        }
    }

    fn coordinates(self, point: Vec3) -> [f64; 2] {
        match self {
            Self::Cylinder(info) => cylinder_coordinates(info, point),
            Self::Cone(info) => cone_coordinates(info, point),
            Self::Sphere(info) => sphere_coordinates(info, point),
            Self::Torus(info) => torus_coordinates(info, point),
        }
    }

    fn station(self, point: Vec3) -> f64 {
        match self {
            Self::Cylinder(info) => station(info.carrier, point),
            Self::Cone(info) => cone_station(info.carrier, point),
            Self::Sphere(info) => sphere_station(info, point),
            Self::Torus(info) => point.sub(info.carrier.center).dot(info.axis),
        }
    }

    /// The exact topology tolerance expressed along the angular parameter:
    /// an arc that starts at the seam ruling reaches it through a second
    /// chain whose fitted radial direction differs by the carrier noise.
    fn seam_tolerance(self, tolerance: f64) -> f64 {
        match self {
            Self::Cylinder(info) => (tolerance / (TAU * info.carrier.radius)).max(1.0e-8),
            Self::Cone(_) => 1.0e-8,
            Self::Sphere(info) => (tolerance / (TAU * info.carrier.radius)).max(1.0e-8),
            Self::Torus(info) => (tolerance / (TAU * info.carrier.major_radius)).max(1.0e-8),
        }
    }

    /// Radial bar for a source vertex on a ring or arc chain: the exact
    /// topology tolerance, or the deviation the segmenter already accepted
    /// for this carrier when that is wider. A vertex the tessellator left
    /// on a rim chord is still on the ring the exact circle replaces.
    fn radial_bar(self, tolerance: f64) -> f64 {
        match self {
            Self::Cylinder(info) => accepted_radial_bar(tolerance, info.carrier.deviation),
            Self::Cone(info) => accepted_radial_bar(tolerance, info.carrier.deviation),
            Self::Sphere(info) => accepted_radial_bar(tolerance, info.carrier.deviation),
            Self::Torus(info) => accepted_radial_bar(tolerance, info.carrier.deviation),
        }
    }

    fn radial_error(self, point: Vec3) -> f64 {
        match self {
            Self::Cylinder(info) => {
                (radial(info.carrier, point).length() - info.carrier.radius).abs()
            }
            Self::Cone(info) => {
                let s = cone_station(info.carrier, point);
                (cone_radial(info.carrier, point).length() - s * info.carrier.half_angle.tan())
                    .abs()
                    * info.carrier.half_angle.cos()
            }
            Self::Sphere(info) => sphere_radial_error(info.carrier, point),
            Self::Torus(info) => torus_radial_error(info.carrier, point),
        }
    }
}

fn revolve_info<'a>(
    region: u32,
    cylinders: &'a HashMap<u32, CylinderInfo>,
    cones: &'a HashMap<u32, ConeInfo>,
    spheres: &'a HashMap<u32, SphereInfo>,
    tori: &'a HashMap<u32, TorusInfo>,
) -> Option<RevolveInfoRef<'a>> {
    cylinders
        .get(&region)
        .map(RevolveInfoRef::Cylinder)
        .or_else(|| cones.get(&region).map(RevolveInfoRef::Cone))
        .or_else(|| spheres.get(&region).map(RevolveInfoRef::Sphere))
        .or_else(|| tori.get(&region).map(RevolveInfoRef::Torus))
}

#[allow(clippy::too_many_arguments)]
fn build_chain_edge(
    chain_index: usize,
    chain: &Chain,
    vertices: &[Vec3],
    planes: &HashMap<u32, PlaneInfo>,
    cylinders: &HashMap<u32, CylinderInfo>,
    cones: &HashMap<u32, ConeInfo>,
    spheres: &HashMap<u32, SphereInfo>,
    tori: &HashMap<u32, TorusInfo>,
    tolerance: f64,
    switches: HybridSwitches,
    common_station: &HashMap<usize, f64>,
    moved_line_ends: &mut usize,
    arena: &mut TopologyArena,
    arena_vertices: &mut HashMap<usize, brep_kernel::VertexId>,
    ids: &mut Ids,
) -> Result<BuiltEdge, String> {
    let points = chain_points(chain, vertices);
    let mut custom_revolve_pcurves = HashMap::new();
    let revolve_ids = [chain.adjacent.0, chain.adjacent.1]
        .into_iter()
        .filter(|id| {
            cylinders.contains_key(id)
                || cones.contains_key(id)
                || spheres.contains_key(id)
                || tori.contains_key(id)
        })
        .collect::<Vec<_>>();
    if revolve_ids.len() > 2 {
        return Err(format!(
            "hybrid BREP: chain {chain_index} has too many revolved neighbors"
        ));
    }
    // An open arc on a sphere patch takes its curve from the other revolve;
    // the sphere side is fitted to it below.
    let mut revolve_ids = revolve_ids;
    if !chain.closed && revolve_ids.len() == 2 && spheres.contains_key(&revolve_ids[0]) {
        revolve_ids.swap(0, 1);
    }
    let (curve, t0, t1, curve_along_chain, primary_uv) = if let Some(&region) = revolve_ids.first()
    {
        let info = revolve_info(region, cylinders, cones, spheres, tori).unwrap();
        let stations = points.iter().map(|&p| info.station(p)).collect::<Vec<_>>();
        let station_spread = stations.iter().copied().fold(f64::NEG_INFINITY, f64::max)
            - stations.iter().copied().fold(f64::INFINITY, f64::min);
        // Lane V row 2b (HybridSwitches::ring_mean_bar).
        let station_error = if switches.ring_mean_bar
            && matches!(info, RevolveInfoRef::Cylinder(_) | RevolveInfoRef::Cone(_))
        {
            let mean = stations.iter().sum::<f64>() / stations.len() as f64;
            stations.iter().map(|station| (station - mean).abs()).fold(0.0, f64::max)
        } else {
            station_spread
        };
        let radius_error = points
            .iter()
            .map(|&point| info.radial_error(point))
            .fold(0.0, f64::max);
        let mut coordinates = points
            .iter()
            .map(|&p| info.coordinates(p))
            .collect::<Vec<_>>();
        let mut us = coordinates.iter().map(|uv| uv[0]).collect::<Vec<_>>();
        unwrap(&mut us);
        for (uv, u) in coordinates.iter_mut().zip(&us) {
            uv[0] = *u;
        }
        let tilted_plane = match info {
            RevolveInfoRef::Sphere(sphere) if chain.closed && station_spread > tolerance => {
                [chain.adjacent.0, chain.adjacent.1]
                    .into_iter()
                    .find_map(|neighbor| planes.get(&neighbor).map(|plane| (sphere, *plane)))
            }
            _ => None,
        };
        if let Some((sphere, plane)) = tilted_plane {
            let (curve, t0, t1, curve_along_chain, pcurve) =
                tilted_sphere_ring_edge(chain_index, sphere, plane, &points, tolerance)?;
            let domain = pcurve.domain()?;
            let start = pcurve.evaluate(domain[0])?;
            let end = pcurve.evaluate(domain[1])?;
            custom_revolve_pcurves.insert(region, pcurve);
            (
                curve,
                t0,
                t1,
                curve_along_chain,
                Some(ChainUv {
                    region,
                    start: [start.x, start.y],
                    end: [end.x, end.y],
                }),
            )
        } else if chain.closed {
            let first = points[0];
            let mut ring_points = points.clone();
            ring_points.push(first);
            let mut ring_us = ring_points
                .iter()
                .map(|&p| info.coordinates(p)[0])
                .collect::<Vec<_>>();
            unwrap(&mut ring_us);
            let sweep = ring_us[ring_us.len() - 1] - ring_us[0];
            if station_error > tolerance
                || radius_error > info.radial_bar(tolerance)
                || (sweep.abs() - 1.0).abs() > 0.02
                || !monotone_parameter_walk(&ring_us, 1.0e-5)
            {
                return Err(format!(
                    "hybrid BREP: closed chain {chain_index} (region {region}, adjacent {:?}) is not a revolve ring: station spread {station_spread:.3e}, radial error {radius_error:.3e} (bar {:.3e}), sweep {sweep:.6}, monotone {}",
                    chain.adjacent,
                    info.radial_bar(tolerance),
                    monotone_parameter_walk(&ring_us, 1.0e-5)
                ));
            }
            let v = coordinates.iter().map(|uv| uv[1]).sum::<f64>() / coordinates.len() as f64;
            let curve = info.surface().iso_curve_v(v)?;
            (
                curve,
                0.0,
                1.0,
                sweep > 0.0,
                Some(ChainUv {
                    region,
                    start: [0.0, v],
                    end: [if sweep > 0.0 { 1.0 } else { -1.0 }, v],
                }),
            )
        } else if station_error <= tolerance && radius_error <= info.radial_bar(tolerance) {
            if !monotone_parameter_walk(&us, 1.0e-5) {
                return Err(format!(
                    "hybrid BREP: cylinder arc chain {chain_index} backtracks"
                ));
            }
            let mut a = us[0];
            let mut b = *us.last().unwrap();
            // [`HybridSwitches::cone_seam_bar`]: a cone's arc may end within
            // the bar its seam search allowed (tolerance over the wide end's
            // circumference), not 1e-8.
            let seam_tolerance = match info {
                RevolveInfoRef::Cone(cone) if switches.cone_seam_bar => {
                    let radius = (cone.base_station + cone.height) * cone.carrier.half_angle.tan();
                    (tolerance / (TAU * radius.max(tolerance))).max(1.0e-8)
                }
                _ => info.seam_tolerance(tolerance),
            };
            let shift = (-2..=2)
                .map(f64::from)
                .find(|shift| {
                    a + shift >= -seam_tolerance
                        && b + shift >= -seam_tolerance
                        && a + shift <= 1.0 + seam_tolerance
                        && b + shift <= 1.0 + seam_tolerance
                })
                .ok_or_else(|| {
                    format!(
                        "hybrid BREP: cylinder arc chain {chain_index} (region {region}, adjacent {:?}) crosses the chosen seam: u {a:.6} -> {b:.6} over {} points",
                        chain.adjacent,
                        points.len()
                    )
                })?;
            a += shift;
            b += shift;
            if a.abs() <= seam_tolerance {
                a = 0.0;
            } else if (a - 1.0).abs() <= seam_tolerance {
                a = 1.0;
            }
            if b.abs() <= seam_tolerance {
                b = 0.0;
            } else if (b - 1.0).abs() <= seam_tolerance {
                b = 1.0;
            }
            if (b - a).abs() <= 1.0e-9 || (b - a).abs() >= 1.0 - 1.0e-9 {
                return Err(format!(
                    "hybrid BREP: cylinder arc chain {chain_index} has invalid sweep"
                ));
            }
            let v = coordinates.iter().map(|uv| uv[1]).sum::<f64>() / coordinates.len() as f64;
            // [`HybridSwitches::revolve_ends_first`]: arcs of one ring share
            // its station.
            let own = v;
            let v = common_station.get(&chain_index).copied().unwrap_or(v);
            *moved_line_ends += usize::from(v != own);
            let low = a.min(b);
            let high = a.max(b);
            let curve = trim_curve(&info.surface().iso_curve_v(v)?, low, high)?;
            let domain = curve.domain()?;
            (
                curve,
                domain[0],
                domain[1],
                b > a,
                Some(ChainUv {
                    region,
                    start: [a, v],
                    end: [b, v],
                }),
            )
        } else if !matches!(info, RevolveInfoRef::Sphere(_) | RevolveInfoRef::Torus(_))
            && (max_line_deviation(&points) <= tolerance
                || us.iter().copied().fold(f64::NEG_INFINITY, f64::max)
                    - us.iter().copied().fold(f64::INFINITY, f64::min)
                    <= 1.0e-4)
        {
            let a = coordinates[0];
            let mut b = *coordinates.last().unwrap();
            while b[0] - a[0] > 0.5 {
                b[0] -= 1.0;
            }
            while b[0] - a[0] < -0.5 {
                b[0] += 1.0;
            }
            if (b[0] - a[0]).abs() > 1.0e-4 {
                return Err(format!(
                    "hybrid BREP: chain {chain_index} is not an axial ruling"
                ));
            }
            let u = (a[0] + b[0]) * 0.5;
            let surface = info.surface();
            // An isoparametric ruling sits at the mean of its endpoint
            // angles, which moves each endpoint tangentially by half their
            // angular spread.  The arc chains meeting this ruling place
            // those corner vertices at their own angle, and
            // `exact_arena_vertex` compares the two placements at the
            // exact-topology bar; a tessellation whose ruling endpoints
            // differ by a fraction of a degree would lose its whole
            // cylinder to that disagreement.  Half the bar leaves the rest
            // of it to the offsets the arc chain itself carries (its mean
            // station, its seam snap).
            let parametric_offset = [a, b]
                .into_iter()
                .map(|uv| {
                    Ok(surface
                        .evaluate(u.rem_euclid(1.0), uv[1])?
                        .sub(surface.evaluate(uv[0].rem_euclid(1.0), uv[1])?)
                        .length())
                })
                .collect::<Result<Vec<f64>, String>>()?
                .into_iter()
                .fold(0.0, f64::max);
            if parametric_offset > tolerance * 0.5 {
                // Build the segment between the endpoints' own positions on
                // the carrier instead.  It is the chord of a helix spanning
                // at most 1.0e-4 of a turn, so it leaves the surface by
                // radius * (pi * 1.0e-4)^2 / 2 -- under a nanometre at any
                // scale a mesh import carries -- while both corners stay
                // exactly where the arcs that share them put them.
                if max_line_deviation(&points) > tolerance {
                    return Err(format!(
                        "hybrid BREP: chain {chain_index} is neither an axial ruling nor straight"
                    ));
                }
                (
                    make_line(
                        surface.evaluate(a[0].rem_euclid(1.0), a[1])?,
                        surface.evaluate(b[0].rem_euclid(1.0), b[1])?,
                    )?,
                    0.0,
                    1.0,
                    true,
                    Some(ChainUv {
                        region,
                        start: a,
                        end: b,
                    }),
                )
            } else {
                let low = a[1].min(b[1]);
                let high = a[1].max(b[1]);
                let curve = trim_curve(&surface.iso_curve_u(u.rem_euclid(1.0))?, low, high)?;
                let domain = curve.domain()?;
                (
                    curve,
                    domain[0],
                    domain[1],
                    b[1] > a[1],
                    Some(ChainUv {
                        region,
                        start: [u, a[1]],
                        end: [u, b[1]],
                    }),
                )
            }
        } else if let (Some(cone), Some(plane)) = (
            cones.get(&region),
            [chain.adjacent.0, chain.adjacent.1]
                .into_iter()
                .find_map(|neighbor| planes.get(&neighbor)),
        ) {
            let normal = plane.normal.normalized()?;
            let x = normal.perpendicular()?;
            let y = normal.cross(x).normalized()?;
            let extent = points
                .iter()
                .map(|point| point.sub(plane.origin).length())
                .fold(1.0_f64, f64::max)
                * 4.0;
            let plane_surface = brep_kernel::make_plane(
                plane.origin.sub(x.scale(extent)).sub(y.scale(extent)),
                x,
                y,
                extent * 2.0,
                extent * 2.0,
            )?;
            let branches = intersect_analytic_pair(&plane_surface, &cone.surface, tolerance)
                .filter(|branches| !branches.is_empty())
                .unwrap_or_else(|| {
                    exact_local_cone_plane_conic(cone, *plane, &points, tolerance)
                        .into_iter()
                        .collect()
                });
            let mut candidates = branches
                .into_iter()
                .map(|curve| {
                    let projections = points
                        .iter()
                        .map(|&point| project_point_to_curve(&curve, point))
                        .collect::<Result<Vec<_>, String>>()?;
                    let score: f64 = projections
                        .iter()
                        .map(|projection| projection.distance)
                        .sum();
                    Ok((score, curve, projections))
                })
                .collect::<Result<Vec<_>, String>>()?;
            candidates.sort_by(|a, b| a.0.total_cmp(&b.0));
            let (_, curve, projections) = candidates.into_iter().next().ok_or_else(|| {
                format!("hybrid BREP: exact cone/plane conic is empty for chain {chain_index}")
            })?;
            let source_error = projections
                .iter()
                .map(|projection| projection.distance)
                .fold(0.0_f64, f64::max);
            if source_error > tolerance * 2.0 {
                return Err(format!(
                    "hybrid BREP: exact cone/plane conic chain {chain_index} misses source by {source_error:.6e}"
                ));
            }
            let p0 = projections[0];
            let p1 = *projections.last().unwrap();
            let low = p0.u.min(p1.u);
            let high = p0.u.max(p1.u);
            let direction = (p1.u - p0.u).signum();
            let monotone = direction != 0.0
                && projections.windows(2).all(|pair| {
                    (pair[1].u - pair[0].u) * direction >= -1.0e-8
                        && pair[1].u >= low - 1.0e-8
                        && pair[1].u <= high + 1.0e-8
                });
            if !monotone || high - low <= 1.0e-10 {
                return Err(format!(
                    "hybrid BREP: exact cone/plane conic chain {chain_index} crosses its branch seam"
                ));
            }
            let pcurve = build_pcurve_on_surface_range(
                &cone.surface,
                &curve,
                low,
                high,
                true,
                (tolerance * 0.05).max(1.0e-9),
            )?;
            let pcurve_domain = pcurve.domain()?;
            for sample in 0..=16 {
                let fraction = sample as f64 / 16.0;
                let t = low + (high - low) * fraction;
                let q = pcurve_domain[0] + (pcurve_domain[1] - pcurve_domain[0]) * fraction;
                let edge_point = curve.evaluate(t)?;
                let uv = pcurve.evaluate(q)?;
                let surface_point = cone.surface.evaluate(uv.x, uv.y)?;
                if edge_point.sub(surface_point).length() > tolerance * 0.1
                    || edge_point.sub(plane.origin).dot(normal).abs() > tolerance * 0.1
                {
                    return Err(format!(
                        "hybrid BREP: cone/plane conic chain {chain_index} pcurve failed agreement"
                    ));
                }
            }
            let edge_uv_start = pcurve.evaluate(pcurve_domain[0])?;
            let edge_uv_end = pcurve.evaluate(pcurve_domain[1])?;
            let curve_along_chain = p1.u > p0.u;
            let (chain_uv_start, chain_uv_end) = if curve_along_chain {
                (edge_uv_start, edge_uv_end)
            } else {
                (edge_uv_end, edge_uv_start)
            };
            custom_revolve_pcurves.insert(region, pcurve);
            (
                curve,
                low,
                high,
                curve_along_chain,
                Some(ChainUv {
                    region,
                    start: [chain_uv_start.x, chain_uv_start.y],
                    end: [chain_uv_end.x, chain_uv_end.y],
                }),
            )
        } else if let (Some(first), Some(second)) = (
            cylinders.get(&chain.adjacent.0).filter(|_| !chain.closed),
            cylinders.get(&chain.adjacent.1),
        ) {
            let (curve, t0, t1, curve_along_chain, pcurves) = cylinder_pair_section_edge(
                chain_index,
                [(chain.adjacent.0, first), (chain.adjacent.1, second)],
                &points,
                tolerance,
            )?;
            let mut primary = None;
            for (owner, pcurve) in pcurves {
                let domain = pcurve.domain()?;
                let (start, end) = (pcurve.evaluate(domain[0])?, pcurve.evaluate(domain[1])?);
                let (start, end) = if curve_along_chain { (start, end) } else { (end, start) };
                if owner == region {
                    primary = Some(ChainUv {
                        region,
                        start: [start.x, start.y],
                        end: [end.x, end.y],
                    });
                }
                custom_revolve_pcurves.insert(owner, pcurve);
            }
            (curve, t0, t1, curve_along_chain, primary)
        } else {
            return Err(format!(
                "hybrid BREP: revolved chain {chain_index} (region {region}, adjacent {:?}, closed={}, station spread {station_spread:.6e}, radial error {radius_error:.6e}, line deviation {:.6e}, uv {:?}->{:?}, points {:?}) is neither arc, ring, nor ruling",
                chain.adjacent,
                chain.closed,
                max_line_deviation(&points),
                coordinates.first(),
                coordinates.last(),
                points,
                // Source samples are included to make conservative demotions
                // diagnosable in focused regression tests.
            ));
        }
    } else {
        if chain.closed || max_line_deviation(&points) > tolerance {
            return Err(format!(
                "hybrid BREP: non-cylinder chain {chain_index} is not a line"
            ));
        }
        // [`HybridSwitches::revolve_ends_first`]: a line ends where a
        // revolved chain already placed its end vertex.
        let placed = |vertex: usize, mesh: Vec3| {
            arena_vertices
                .get(&vertex)
                .filter(|_| switches.revolve_ends_first)
                .and_then(|&id| arena.vertices.get(id))
                .map_or(mesh, |placed| placed.point)
        };
        let (start, end) = (
            placed(chain.vertices[0], points[0]),
            placed(*chain.vertices.last().unwrap(), *points.last().unwrap()),
        );
        *moved_line_ends += usize::from(start.sub(points[0]).length() > 0.0) + usize::from(end.sub(*points.last().unwrap()).length() > 0.0);
        (
            make_line(start, end)?,
            0.0,
            1.0,
            true,
            None,
        )
    };

    let first = chain.vertices[0];
    let last = if chain.closed {
        first
    } else {
        *chain.vertices.last().unwrap()
    };
    let (start_mesh, end_mesh) = if curve_along_chain {
        (first, last)
    } else {
        (last, first)
    };
    let exact_start = curve.evaluate(t0)?;
    let exact_end = curve.evaluate(t1)?;
    let start = exact_arena_vertex(
        start_mesh,
        exact_start,
        tolerance,
        arena,
        arena_vertices,
        ids,
    )?;
    let end = exact_arena_vertex(end_mesh, exact_end, tolerance, arena, arena_vertices, ids)?;
    let edge = arena.edges.insert(ArenaEdge {
        wire_id: ids.next(),
        curve,
        t0,
        t1,
        start,
        end,
        degenerate: false,
        name: None,
    });
    if !chain.closed {
        if let Some(sphere) = revolve_ids.get(1).and_then(|secondary| spheres.get(secondary)) {
            let pcurve = sphere_patch_pcurve(
                chain_index,
                sphere,
                &arena.edges[edge].curve,
                t0,
                t1,
                tolerance,
            )?;
            custom_revolve_pcurves.insert(revolve_ids[1], pcurve);
        }
    }
    let mut revolve_uvs = primary_uv.into_iter().collect::<Vec<_>>();
    if let Some(pcurve) = revolve_ids
        .get(1)
        .and_then(|secondary| custom_revolve_pcurves.get(secondary))
    {
        let domain = pcurve.domain()?;
        let (start, end) = (pcurve.evaluate(domain[0])?, pcurve.evaluate(domain[1])?);
        let (start, end) = if curve_along_chain { (start, end) } else { (end, start) };
        revolve_uvs.push(ChainUv {
            region: revolve_ids[1],
            start: [start.x, start.y],
            end: [end.x, end.y],
        });
    } else if revolve_ids.len() == 2 {
        let secondary_region = revolve_ids[1];
        let secondary = revolve_info(secondary_region, cylinders, cones, spheres, tori).unwrap();
        let mut coordinates = points
            .iter()
            .map(|&point| secondary.coordinates(point))
            .collect::<Vec<_>>();
        let mut us = coordinates.iter().map(|uv| uv[0]).collect::<Vec<_>>();
        unwrap(&mut us);
        for (uv, u) in coordinates.iter_mut().zip(&us) {
            uv[0] = *u;
        }
        let uv = if chain.closed {
            let mut ring_points = points.clone();
            ring_points.push(points[0]);
            let mut ring_us = ring_points
                .iter()
                .map(|&point| secondary.coordinates(point)[0])
                .collect::<Vec<_>>();
            unwrap(&mut ring_us);
            let sweep = ring_us.last().unwrap() - ring_us[0];
            let v = coordinates.iter().map(|uv| uv[1]).sum::<f64>() / coordinates.len() as f64;
            ChainUv {
                region: secondary_region,
                start: [0.0, v],
                end: [if sweep > 0.0 { 1.0 } else { -1.0 }, v],
            }
        } else {
            let start = coordinates[0];
            let mut end = *coordinates.last().unwrap();
            if (start[1] - end[1]).abs() <= 1.0e-5 {
                // A common ring arc: constant station on both revolves.
                ChainUv {
                    region: secondary_region,
                    start,
                    end,
                }
            } else {
                // A shared axial ruling: constant angle on the secondary
                // cylinder, which may sit on its parameter seam.
                while end[0] - start[0] > 0.5 {
                    end[0] -= 1.0;
                }
                while end[0] - start[0] < -0.5 {
                    end[0] += 1.0;
                }
                if (end[0] - start[0]).abs() > 1.0e-4 {
                    return Err(format!(
                        "hybrid BREP: shared revolve chain {chain_index} is neither a common ring arc nor a common ruling"
                    ));
                }
                let u = ((start[0] + end[0]) * 0.5).rem_euclid(1.0);
                ChainUv {
                    region: secondary_region,
                    start: [u, start[1]],
                    end: [u, end[1]],
                }
            }
        };
        revolve_uvs.push(uv);
    }
    Ok(BuiltEdge {
        edge,
        curve_along_chain,
        revolve_uvs,
        custom_revolve_pcurves,
    })
}

fn plane_frame(
    plane: PlaneInfo,
    boundary: &RegionBoundary,
    chains: &[Chain],
    vertices: &[Vec3],
) -> Result<(Vec3, Vec3, Vec3, f64, f64), String> {
    let normal = plane.normal.normalized()?;
    let x = normal.perpendicular()?;
    let y = normal.cross(x).normalized()?;
    let mut min_u = f64::INFINITY;
    let mut max_u = f64::NEG_INFINITY;
    let mut min_v = f64::INFINITY;
    let mut max_v = f64::NEG_INFINITY;
    for traversal in boundary.cycles.iter().flatten() {
        for &vertex in &chains[traversal.chain].vertices {
            let delta = vertices[vertex].sub(plane.origin);
            let u = delta.dot(x);
            let v = delta.dot(y);
            min_u = min_u.min(u);
            max_u = max_u.max(u);
            min_v = min_v.min(v);
            max_v = max_v.max(v);
        }
    }
    if !min_u.is_finite() || max_u - min_u <= 1.0e-12 || max_v - min_v <= 1.0e-12 {
        return Err("hybrid BREP: degenerate planar face bounds".into());
    }
    Ok((
        plane.origin.add(x.scale(min_u)).add(y.scale(min_v)),
        x,
        y,
        max_u - min_u,
        max_v - min_v,
    ))
}

fn affine_pcurve(curve: &NurbsCurve, origin: Vec3, x: Vec3, y: Vec3) -> Result<NurbsCurve, String> {
    let controls = curve
        .control_points
        .iter()
        .map(|control| {
            if !control.w.is_finite() || control.w.abs() <= f64::EPSILON {
                return Err("hybrid BREP: pcurve source has an invalid homogeneous weight".into());
            }
            let point = Vec3::new(
                control.x / control.w,
                control.y / control.w,
                control.z / control.w,
            );
            let delta = point.sub(origin);
            Ok(Vec4 {
                x: delta.dot(x) * control.w,
                y: delta.dot(y) * control.w,
                z: 0.0,
                w: control.w,
            })
        })
        .collect::<Result<Vec<_>, String>>()?;
    NurbsCurve::new(curve.degree, curve.knots.clone(), controls)
}

#[allow(clippy::too_many_arguments)]
fn build_plane_face(
    _region: u32,
    plane: PlaneInfo,
    boundary: &RegionBoundary,
    chains: &[Chain],
    built_edges: &[BuiltEdge],
    vertices: &[Vec3],
    arena: &mut TopologyArena,
    ids: &mut Ids,
) -> Result<FaceId, String> {
    let (origin, x, y, u_extent, v_extent) = plane_frame(plane, boundary, chains, vertices)?;
    let surface = brep_kernel::make_plane(origin, x, y, u_extent, v_extent)?;
    let mut loops = Vec::new();
    for cycle in &boundary.cycles {
        let mut coedges = Vec::new();
        for traversal in cycle {
            let built = &built_edges[traversal.chain];
            let edge = &arena.edges[built.edge];
            let mut pcurve = affine_pcurve(&edge.curve, origin, x, y)?;
            let forward = traversal.forward == built.curve_along_chain;
            if !forward {
                pcurve = pcurve.reversed()?;
            }
            coedges.push(arena.coedges.insert(ArenaCoedge {
                wire_id: ids.next(),
                edge: built.edge,
                forward,
                pcurve,
            }));
        }
        loops.push(arena.loops.insert(ArenaLoop {
            wire_id: ids.next(),
            coedges,
        }));
    }
    Ok(arena.faces.insert(ArenaFace {
        wire_id: ids.next(),
        surface,
        same_sense: true,
        loops,
        name: None,
    }))
}

fn parameter_line(start: [f64; 2], end: [f64; 2]) -> Result<NurbsCurve, String> {
    make_line(
        Vec3::new(start[0], start[1], 0.0),
        Vec3::new(end[0], end[1], 0.0),
    )
}

fn build_cylinder_face(
    region: u32,
    cylinder: &CylinderInfo,
    boundary: &RegionBoundary,
    built_edges: &[BuiltEdge],
    arena: &mut TopologyArena,
    ids: &mut Ids,
) -> Result<FaceId, String> {
    let mut loops = Vec::new();
    for cycle in &boundary.cycles {
        let mut coedges = Vec::new();
        let mut previous_end: Option<[f64; 2]> = None;
        for traversal in cycle {
            let built = &built_edges[traversal.chain];
            let uv = built
                .revolve_uvs
                .iter()
                .find(|uv| uv.region == region)
                .copied()
                .ok_or_else(|| format!("hybrid BREP: cylinder {region} has non-parametric edge"))?;
            let (mut start, mut end) = if traversal.forward {
                (uv.start, uv.end)
            } else {
                (uv.end, uv.start)
            };
            if let Some(previous) = previous_end {
                let shift = (previous[0] - start[0]).round();
                start[0] += shift;
                end[0] += shift;
            }
            previous_end = Some(end);
            let forward = traversal.forward == built.curve_along_chain;
            let mut pcurve = built
                .custom_revolve_pcurves
                .get(&region)
                .cloned()
                .unwrap_or(parameter_line(start, end)?);
            if !forward && built.custom_revolve_pcurves.contains_key(&region) {
                pcurve = pcurve.reversed()?;
            }
            coedges.push(arena.coedges.insert(ArenaCoedge {
                wire_id: ids.next(),
                edge: built.edge,
                forward,
                pcurve,
            }));
        }
        loops.push(arena.loops.insert(ArenaLoop {
            wire_id: ids.next(),
            coedges,
        }));
    }
    Ok(arena.faces.insert(ArenaFace {
        wire_id: ids.next(),
        surface: cylinder.surface.clone(),
        same_sense: cylinder.carrier.sense == 1,
        loops,
        name: None,
    }))
}

/// A sphere face from its ring loops.  A cap around the north pole also
/// carries the surface seam from the ring to the pole and the degenerate
/// pole edge, the way the kernel's own sphere solid is built, so its
/// parameter-space loop closes.
fn build_sphere_face(
    region: u32,
    sphere: &SphereInfo,
    boundary: &RegionBoundary,
    built_edges: &[BuiltEdge],
    arena: &mut TopologyArena,
    ids: &mut Ids,
) -> Result<FaceId, String> {
    let mut loops = Vec::new();
    for cycle in &boundary.cycles {
        let mut coedges = Vec::new();
        let mut previous_end: Option<[f64; 2]> = None;
        let mut ring: Option<(brep_kernel::VertexId, [f64; 2], [f64; 2])> = None;
        for traversal in cycle {
            let built = &built_edges[traversal.chain];
            let uv = built
                .revolve_uvs
                .iter()
                .find(|uv| uv.region == region)
                .copied()
                .ok_or_else(|| format!("hybrid BREP: sphere {region} has non-parametric edge"))?;
            let (mut start, mut end) = if traversal.forward {
                (uv.start, uv.end)
            } else {
                (uv.end, uv.start)
            };
            if let Some(previous) = previous_end {
                let shift = (previous[0] - start[0]).round();
                start[0] += shift;
                end[0] += shift;
            }
            previous_end = Some(end);
            let forward = traversal.forward == built.curve_along_chain;
            let edge = &arena.edges[built.edge];
            ring = Some((edge.start, start, end));
            // A tilted ring carries its fitted loop, oriented along its curve.
            let pcurve = match built.custom_revolve_pcurves.get(&region) {
                Some(pcurve) if forward => pcurve.clone(),
                Some(pcurve) => pcurve.reversed()?,
                None => parameter_line(start, end)?,
            };
            coedges.push(arena.coedges.insert(ArenaCoedge {
                wire_id: ids.next(),
                edge: built.edge,
                forward,
                pcurve,
            }));
        }
        if sphere.north_pole_inside {
            let Some((ring_vertex, ring_start, ring_end)) = ring.filter(|_| cycle.len() == 1) else {
                return Err(format!(
                    "hybrid BREP: sphere {region} cap boundary is not a single ring"
                ));
            };
            let v = ring_start[1];
            let seam = trim_curve(&sphere.surface.iso_curve_u(0.0)?, v, 1.0)?;
            let domain = seam.domain()?;
            let pole_point = sphere.surface.evaluate(0.0, 1.0)?;
            let pole = arena.vertices.insert(ArenaVertex {
                wire_id: ids.next(),
                point: pole_point,
            });
            let seam_edge = arena.edges.insert(ArenaEdge {
                wire_id: ids.next(),
                curve: seam,
                t0: domain[0],
                t1: domain[1],
                start: ring_vertex,
                end: pole,
                degenerate: false,
                name: None,
            });
            let pole_edge = arena.edges.insert(ArenaEdge {
                wire_id: ids.next(),
                curve: make_line(pole_point, pole_point)?,
                t0: 0.0,
                t1: 1.0,
                start: pole,
                end: pole,
                degenerate: true,
                name: None,
            });
            // Up the seam on the ring's far side, across the pole, down the
            // seam on the ring's near side: the loop closes in parameter
            // space with the cap on its left.
            coedges.push(arena.coedges.insert(ArenaCoedge {
                wire_id: ids.next(),
                edge: seam_edge,
                forward: true,
                pcurve: parameter_line([ring_end[0], v], [ring_end[0], 1.0])?,
            }));
            coedges.push(arena.coedges.insert(ArenaCoedge {
                wire_id: ids.next(),
                edge: pole_edge,
                forward: true,
                pcurve: parameter_line([ring_end[0], 1.0], [ring_start[0], 1.0])?,
            }));
            coedges.push(arena.coedges.insert(ArenaCoedge {
                wire_id: ids.next(),
                edge: seam_edge,
                forward: false,
                pcurve: parameter_line([ring_start[0], 1.0], [ring_start[0], v])?,
            }));
        }
        loops.push(arena.loops.insert(ArenaLoop {
            wire_id: ids.next(),
            coedges,
        }));
    }
    Ok(arena.faces.insert(ArenaFace {
        wire_id: ids.next(),
        surface: sphere.surface.clone(),
        same_sense: sphere.carrier.sense == 1,
        loops,
        name: None,
    }))
}

/// A revolve face whose loops are all parameter lines of its surface (a
/// torus band between two latitude rings, as a full cylinder wall is
/// between two circles): each loop's pcurves are the chains' own uv lines.
#[allow(clippy::too_many_arguments)]
fn build_revolve_band_face(
    region: u32,
    kind: &str,
    surface: &NurbsSurface,
    same_sense: bool,
    boundary: &RegionBoundary,
    built_edges: &[BuiltEdge],
    arena: &mut TopologyArena,
    ids: &mut Ids,
) -> Result<FaceId, String> {
    let mut loops = Vec::new();
    for cycle in &boundary.cycles {
        let mut coedges = Vec::new();
        let mut previous_end: Option<[f64; 2]> = None;
        for traversal in cycle {
            let built = &built_edges[traversal.chain];
            if built.custom_revolve_pcurves.contains_key(&region) {
                return Err(format!("hybrid BREP: {kind} {region} has a fitted-pcurve edge"));
            }
            let uv = built
                .revolve_uvs
                .iter()
                .find(|uv| uv.region == region)
                .copied()
                .ok_or_else(|| format!("hybrid BREP: {kind} {region} has non-parametric edge"))?;
            let (mut start, mut end) = if traversal.forward {
                (uv.start, uv.end)
            } else {
                (uv.end, uv.start)
            };
            if let Some(previous) = previous_end {
                let shift = (previous[0] - start[0]).round();
                start[0] += shift;
                end[0] += shift;
            }
            previous_end = Some(end);
            coedges.push(arena.coedges.insert(ArenaCoedge {
                wire_id: ids.next(),
                edge: built.edge,
                forward: traversal.forward == built.curve_along_chain,
                pcurve: parameter_line(start, end)?,
            }));
        }
        loops.push(arena.loops.insert(ArenaLoop {
            wire_id: ids.next(),
            coedges,
        }));
    }
    Ok(arena.faces.insert(ArenaFace {
        wire_id: ids.next(),
        surface: surface.clone(),
        same_sense,
        loops,
        name: None,
    }))
}

/// Tube angle of a point about a torus frame, unwrapped around the band's
/// middle: in [0, 2π] for every point of a band clear of the tube seam.
fn torus_tube_angle(center: Vec3, axis: Vec3, major_radius: f64, middle: f64, point: Vec3) -> f64 {
    let d = point.sub(center);
    let z = d.dot(axis);
    let rho = d.sub(axis.scale(z)).length();
    let raw = z.atan2(rho - major_radius);
    let delta = (raw - middle + std::f64::consts::PI).rem_euclid(TAU) - std::f64::consts::PI;
    middle + delta
}

fn torus_coordinates(info: &TorusInfo, point: Vec3) -> [f64; 2] {
    let d = point.sub(info.carrier.center);
    let radial = d.sub(info.axis.scale(d.dot(info.axis)));
    let angle = radial.dot(info.y_axis).atan2(radial.dot(info.x_axis)).rem_euclid(TAU);
    let u = circle_angle_to_parameter(4, TAU, angle);
    let tube = torus_tube_angle(
        info.carrier.center,
        info.axis,
        info.carrier.major_radius,
        info.band_middle,
        point,
    )
    .clamp(0.0, TAU);
    [u, circle_angle_to_parameter(4, TAU, tube)]
}

/// Frame each torus band on the kernel's torus template. The band must be
/// bounded by closed rings only; its axis snaps onto a coaxial cylinder or
/// cone neighbour's and takes that neighbour's seam meridian (a shared ring
/// is then one parameter line on both faces), and it is turned so that its
/// tube angles stay clear of the template's tube seam at the outer equator.
#[allow(clippy::too_many_arguments)]
fn build_torus_surfaces(
    carriers: &HashMap<u32, TorusCarrier>,
    cylinders: &HashMap<u32, CylinderInfo>,
    cones: &HashMap<u32, ConeInfo>,
    chains: &[Chain],
    vertices: &[Vec3],
    region_samples: &HashMap<u32, Vec<Vec3>>,
    tolerance: f64,
) -> Result<HashMap<u32, TorusInfo>, (u32, String)> {
    let mut result = HashMap::new();
    for (&region, &original) in carriers {
        let relevant = chains
            .iter()
            .filter(|chain| chain.adjacent.0 == region || chain.adjacent.1 == region)
            .collect::<Vec<_>>();
        if relevant.is_empty() || relevant.iter().any(|chain| !chain.closed) {
            return Err((
                region,
                format!("hybrid BREP: torus {region} is not bounded by closed rings"),
            ));
        }
        let mut carrier = original;
        let mut seam = None;
        for chain in &relevant {
            let neighbor = if chain.adjacent.0 == region {
                chain.adjacent.1
            } else {
                chain.adjacent.0
            };
            let frame = cylinders
                .get(&neighbor)
                .map(|info| (info.carrier.origin, info.carrier.axis, info.x_axis))
                .or_else(|| {
                    cones
                        .get(&neighbor)
                        .map(|info| (info.carrier.apex, info.carrier.axis, info.x_axis))
                });
            if let Some((origin, axis, x_axis)) = frame {
                let axis = if axis.dot(carrier.axis) < 0.0 {
                    axis.scale(-1.0)
                } else {
                    axis
                };
                carrier.axis = axis;
                carrier.center = origin.add(axis.scale(carrier.center.sub(origin).dot(axis)));
                seam = Some(x_axis);
                break;
            }
        }
        // The band's tube angles: its samples and its ring points.
        let mut points = region_samples.get(&region).cloned().unwrap_or_default();
        for chain in &relevant {
            points.extend(chain_points(chain, vertices));
        }
        if points.is_empty() {
            return Err((region, format!("hybrid BREP: torus {region} has no samples")));
        }
        let mut frame = None;
        for sign in [1.0, -1.0] {
            let axis = carrier.axis.scale(sign);
            let (sin, cos) = points.iter().fold((0.0, 0.0), |(s, c), &point| {
                let raw =
                    torus_tube_angle(carrier.center, axis, carrier.major_radius, 0.0, point);
                (s + raw.sin(), c + raw.cos())
            });
            let middle = sin.atan2(cos).rem_euclid(TAU);
            let angular = tolerance / carrier.minor_radius;
            let clear = points.iter().all(|&point| {
                let angle =
                    torus_tube_angle(carrier.center, axis, carrier.major_radius, middle, point);
                angle >= -angular && angle <= TAU + angular
            });
            if clear {
                frame = Some((axis, middle));
                break;
            }
        }
        let Some((axis, band_middle)) = frame else {
            return Err((
                region,
                format!("hybrid BREP: torus {region} band crosses the tube seam either way"),
            ));
        };
        let x_axis = seam
            .map(|x| x.sub(axis.scale(x.dot(axis))))
            .unwrap_or_else(|| axis.perpendicular().unwrap_or_default())
            .normalized()
            .map_err(|error| (region, error))?;
        let y_axis = axis.cross(x_axis).normalized().map_err(|error| (region, error))?;
        let tube = make_arc(
            carrier.center.add(x_axis.scale(carrier.major_radius)),
            x_axis,
            axis,
            carrier.minor_radius,
            0.0,
            TAU,
        )
        .map_err(|error| (region, error))?;
        let surface = make_revolution(carrier.center, axis, &tube, TAU)
            .map_err(|error| (region, error))?;
        result.insert(
            region,
            TorusInfo {
                carrier,
                axis,
                x_axis,
                y_axis,
                band_middle,
                surface,
            },
        );
    }
    Ok(result)
}

fn build_cone_face(
    region: u32,
    cone: &ConeInfo,
    boundary: &RegionBoundary,
    built_edges: &[BuiltEdge],
    arena: &mut TopologyArena,
    ids: &mut Ids,
) -> Result<FaceId, String> {
    let mut loops = Vec::new();
    for cycle in &boundary.cycles {
        let mut coedges = Vec::new();
        let mut previous_end: Option<[f64; 2]> = None;
        let mut ring: Option<(brep_kernel::VertexId, [f64; 2], [f64; 2])> = None;
        for traversal in cycle {
            let built = &built_edges[traversal.chain];
            let uv = built
                .revolve_uvs
                .iter()
                .find(|uv| uv.region == region)
                .copied()
                .ok_or_else(|| format!("hybrid BREP: cone {region} has non-parametric edge"))?;
            let (mut start, mut end) = if traversal.forward {
                (uv.start, uv.end)
            } else {
                (uv.end, uv.start)
            };
            if let Some(previous) = previous_end {
                let shift = (previous[0] - start[0]).round();
                start[0] += shift;
                end[0] += shift;
            }
            previous_end = Some(end);
            let forward = traversal.forward == built.curve_along_chain;
            let mut pcurve = built
                .custom_revolve_pcurves
                .get(&region)
                .cloned()
                .unwrap_or(parameter_line(start, end)?);
            if !forward && built.custom_revolve_pcurves.contains_key(&region) {
                pcurve = pcurve.reversed()?;
            }
            ring = Some((arena.edges[built.edge].start, start, end));
            coedges.push(arena.coedges.insert(ArenaCoedge {
                wire_id: ids.next(),
                edge: built.edge,
                forward,
                pcurve,
            }));
        }
        if cone.pointed {
            let Some((ring_vertex, ring_start, ring_end)) = ring.filter(|_| cycle.len() == 1) else {
                return Err(format!(
                    "hybrid BREP: pointed cone {region} boundary is not a single ring"
                ));
            };
            let v = ring_start[1];
            let u = ring_start[0].rem_euclid(1.0);
            // Seam from the apex up to the ring vertex, and the apex itself
            // as a degenerate edge along v = 0.
            let seam = trim_curve(&cone.surface.iso_curve_u(u)?, 0.0, v)?;
            let domain = seam.domain()?;
            let apex_point = cone.surface.evaluate(u, 0.0)?;
            let apex = arena.vertices.insert(ArenaVertex {
                wire_id: ids.next(),
                point: apex_point,
            });
            let seam_edge = arena.edges.insert(ArenaEdge {
                wire_id: ids.next(),
                curve: seam,
                t0: domain[0],
                t1: domain[1],
                start: apex,
                end: ring_vertex,
                degenerate: false,
                name: None,
            });
            let apex_edge = arena.edges.insert(ArenaEdge {
                wire_id: ids.next(),
                curve: make_line(apex_point, apex_point)?,
                t0: 0.0,
                t1: 1.0,
                start: apex,
                end: apex,
                degenerate: true,
                name: None,
            });
            // Down the seam on the ring's far side, across the apex, up the
            // seam on the ring's near side.
            coedges.push(arena.coedges.insert(ArenaCoedge {
                wire_id: ids.next(),
                edge: seam_edge,
                forward: false,
                pcurve: parameter_line([ring_end[0], v], [ring_end[0], 0.0])?,
            }));
            coedges.push(arena.coedges.insert(ArenaCoedge {
                wire_id: ids.next(),
                edge: apex_edge,
                forward: true,
                pcurve: parameter_line([ring_end[0], 0.0], [ring_start[0], 0.0])?,
            }));
            coedges.push(arena.coedges.insert(ArenaCoedge {
                wire_id: ids.next(),
                edge: seam_edge,
                forward: true,
                pcurve: parameter_line([ring_start[0], 0.0], [ring_start[0], v])?,
            }));
        }
        loops.push(arena.loops.insert(ArenaLoop {
            wire_id: ids.next(),
            coedges,
        }));
    }
    Ok(arena.faces.insert(ArenaFace {
        wire_id: ids.next(),
        surface: cone.surface.clone(),
        same_sense: cone.carrier.sense == 1,
        loops,
        name: None,
    }))
}

fn sort_region_cycles(
    boundaries: &mut [RegionBoundary],
    planes: &HashMap<u32, PlaneInfo>,
    cylinders: &HashMap<u32, CylinderInfo>,
    cones: &HashMap<u32, ConeInfo>,
    spheres: &HashMap<u32, SphereInfo>,
    tori: &HashMap<u32, TorusInfo>,
    chains: &[Chain],
    vertices: &[Vec3],
) {
    for (region, boundary) in boundaries.iter_mut().enumerate() {
        let mut scored = boundary
            .cycles
            .drain(..)
            .map(|cycle| {
                let mut points = Vec::new();
                for traversal in &cycle {
                    let chain = &chains[traversal.chain];
                    let iter: Box<dyn Iterator<Item = &usize>> = if traversal.forward {
                        Box::new(chain.vertices.iter())
                    } else {
                        Box::new(chain.vertices.iter().rev())
                    };
                    for vertex in iter {
                        points.push(vertices[*vertex]);
                    }
                }
                let area = if let Some(plane) = planes.get(&(region as u32)) {
                    let normal = plane.normal.normalized().unwrap_or(plane.normal);
                    let x = normal.perpendicular().unwrap_or(Vec3::new(1.0, 0.0, 0.0));
                    let y = normal.cross(x);
                    polygon_area(
                        &points
                            .iter()
                            .map(|p| {
                                let d = p.sub(plane.origin);
                                [d.dot(x), d.dot(y)]
                            })
                            .collect::<Vec<_>>(),
                    )
                } else if let Some(cylinder) = cylinders.get(&(region as u32)) {
                    let mut uv = points
                        .iter()
                        .map(|&p| cylinder_coordinates(cylinder, p))
                        .collect::<Vec<_>>();
                    let mut us = uv.iter().map(|p| p[0]).collect::<Vec<_>>();
                    unwrap(&mut us);
                    for (p, u) in uv.iter_mut().zip(us) {
                        p[0] = u;
                    }
                    polygon_area(&uv)
                } else if let Some(cone) = cones.get(&(region as u32)) {
                    let mut uv = points
                        .iter()
                        .map(|&p| cone_coordinates(cone, p))
                        .collect::<Vec<_>>();
                    let mut us = uv.iter().map(|p| p[0]).collect::<Vec<_>>();
                    unwrap(&mut us);
                    for (p, u) in uv.iter_mut().zip(us) {
                        p[0] = u;
                    }
                    polygon_area(&uv)
                } else if let Some(sphere) = spheres.get(&(region as u32)) {
                    let mut uv = points
                        .iter()
                        .map(|&p| sphere_coordinates(sphere, p))
                        .collect::<Vec<_>>();
                    let mut us = uv.iter().map(|p| p[0]).collect::<Vec<_>>();
                    unwrap(&mut us);
                    for (p, u) in uv.iter_mut().zip(us) {
                        p[0] = u;
                    }
                    polygon_area(&uv)
                } else if let Some(torus) = tori.get(&(region as u32)) {
                    let mut uv = points
                        .iter()
                        .map(|&p| torus_coordinates(torus, p))
                        .collect::<Vec<_>>();
                    let mut us = uv.iter().map(|p| p[0]).collect::<Vec<_>>();
                    unwrap(&mut us);
                    for (p, u) in uv.iter_mut().zip(us) {
                        p[0] = u;
                    }
                    polygon_area(&uv)
                } else {
                    0.0
                };
                let same = planes.contains_key(&(region as u32))
                    || cylinders
                        .get(&(region as u32))
                        .is_some_and(|c| c.carrier.sense == 1)
                    || cones
                        .get(&(region as u32))
                        .is_some_and(|c| c.carrier.sense == 1)
                    || spheres
                        .get(&(region as u32))
                        .is_some_and(|c| c.carrier.sense == 1)
                    || tori
                        .get(&(region as u32))
                        .is_some_and(|c| c.carrier.sense == 1);
                let score = if same { area } else { -area };
                (score, cycle)
            })
            .collect::<Vec<_>>();
        scored.sort_by(|a, b| b.0.total_cmp(&a.0));
        boundary.cycles = scored.into_iter().map(|(_, cycle)| cycle).collect();
    }
}

fn polygon_area(points: &[[f64; 2]]) -> f64 {
    if points.len() < 3 {
        return 0.0;
    }
    (0..points.len())
        .map(|i| {
            let a = points[i];
            let b = points[(i + 1) % points.len()];
            a[0] * b[1] - b[0] * a[1]
        })
        .sum::<f64>()
        * 0.5
}

