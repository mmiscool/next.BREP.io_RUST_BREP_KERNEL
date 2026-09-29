use super::*;

// Resolver coordinates are already in millimetres. Identity repair uses the
// existing absolute floor: neither world position nor unrelated large features
// justify merging separate vertices or collapsing an edge. Widening this band
// requires bounded, measured representation error (not a model-size heuristic).
const IMPORT_IDENTITY_TOLERANCE: f64 = 1e-7;

struct SolidBuilder<'a> {
    resolver: &'a Resolver<'a>,
    next_id: u64,
    vertices: Vec<VertexRecord>,
    edges: Vec<EdgeRecord>,
    vertex_of_ref: HashMap<usize, u64>,
    edge_of_ref: HashMap<usize, u64>,
    /// FACETED_BREP welding: an implicit polygon edge is shared by the two faces
    /// that reference the same consecutive point pair, keyed by unordered vertex
    /// ids so the two coedges reuse (and oppositely orient) one `EdgeRecord`.
    edge_of_vertex_pair: HashMap<(u64, u64), u64>,
    /// id -> index into `vertices` / `edges`, kept in lock-step with every push
    /// (via `push_vertex`/`push_edge`) and the single `split_edge_at` insert
    /// (via `rebuild_edge_index`). `vertex_point`/`edge_record` were O(N) linear
    /// scans called per coedge; these make them O(1) and yield the same record
    /// (records are only appended and mutated in place, never reordered).
    vertex_index: HashMap<u64, usize>,
    edge_index: HashMap<u64, usize>,
    /// edge id -> the STEP `EDGE_CURVE.edge_geometry` reference it was built
    /// from, which is the entity that carries any supplied pcurve bundle.
    /// Absent for a FACETED_BREP polygon edge (there is no curve entity) and
    /// for a synthesized seam/pole edge.
    curve_ref_of_edge: HashMap<u64, usize>,
    /// What the supplied-pcurve lane did on this body, reported under
    /// `BREP_DEBUG_SUPPLIED_PCURVE`.
    supplied_report: SuppliedReport,
    /// Every derived trim accepted at a residual over the fit's bar because
    /// the file's curve is not on its carrier (`readings::BoundedTrim`);
    /// returned beside the solid and named on the import report.
    bounded: Vec<builder::readings::BoundedTrim>,
    /// Per-edge provenance for [`crate::import_step_trim_readings`]; `None` on
    /// every ordinary import.
    readings: Option<readings::TrimCapture>,
}

/// Per-body tally of the supplied-pcurve lane. A supplied pcurve is an
/// improvement on the projection fit the importer performs anyway, so every
/// outcome here is "used it" or "fell back"; none of them is an error.
#[derive(Default, Clone, Debug)]
pub(super) struct SuppliedReport {
    /// Coedges whose edge carried a bundle naming this face's carrier.
    pub(super) offered: usize,
    /// Coedges whose pcurve came from the supplied trim.
    pub(super) used: usize,
    /// Offered but declined because the fitted image left the verification band.
    pub(super) declined_residual: usize,
    /// Offered but declined because the snapped stations did not reach the
    /// edge's own vertices — a sub-range edge, or a bundle whose pcurve covers
    /// a different span of a shared curve.
    pub(super) declined_endpoints: usize,
    /// Offered but the fit itself failed.
    pub(super) declined_failed: usize,
    /// Offered, in band, but the edge's own 3D curve already lies ON this
    /// carrier, so it is the geometry of record for the trim and projecting it
    /// reads that accurately. The ordinary outcome on anything our own writer
    /// produced, and the reason a round trip through it stays lossless.
    pub(super) declined_curve_on_carrier: usize,
    /// Worst |supplied trim − edge's 3D curve| seen over accepted stations: the
    /// vendor's own disagreement between its two statements of one edge.
    pub(super) worst_supplied_vs_curve: f64,
    /// Worst coedge-image deviation measured on a DECLINED coedge, and the band
    /// it was held to. This is the number that says how far a file's supplied
    /// trim actually departs from the 3D curve it bounds — the accepted lane
    /// can only ever report disagreements the band already tolerated, so
    /// without this the tally cannot distinguish "no file disagrees" from
    /// "every disagreement was refused".
    pub(super) worst_declined_residual: f64,
    pub(super) declined_band: f64,
}

/// A face resolved to its carrier surface and ordered coedge bounds, with
/// pcurve/loop assembly deferred so a global seam pre-pass can run in between.
struct PendingFace {
    /// The STEP face entity this pending face came from, for error messages.
    /// A face minted by the planar split keeps its source's reference.
    face_ref: usize,
    surface: NurbsSurface,
    /// The STEP surface entity this face's carrier was built from — the
    /// `basis_surface` a supplied pcurve names when it belongs to THIS face.
    surface_ref: usize,
    same_sense: bool,
    /// `(ordered (edge_id, forward) coedges, is_outer_bound)`, outer bounds first.
    bounds: Vec<(Vec<(u64, bool)>, bool)>,
}

mod collect;
pub(in crate::step_import) mod readings;
mod supplied_fit;
mod seams;
pub(in crate::step_import) mod planar_split;
mod rims;
pub(in crate::step_import) mod loops;
mod solids;

pub(super) use solids::*;
