//! Wire harness — the document's `wireHarness` block, the SIDED port graph,
//! per-connection routing, and the bundle solids. Solved at the tail of every
//! history run (like `assembly`), against the ports and spline segments the
//! run published.
//!
//! # The `wireHarness` block
//!
//! `{ connections: [{ id, name, from, to, diameter }], idCounter, buildBundles,
//! cutMargin }` on the history request, round-tripped through the saved
//! document. `from` / `to` are PORT FEATURE IDS (references are ids, never
//! labels — the panel shows the ports' `portName`s and stores their ids).
//! `idCounter` mints `wire-N` ids monotonically, like the feature counter.
//! `buildBundles` (default true) switches the bundle solids off while keeping
//! the routing. `cutMargin` (default 0, omitted at 0) is the amount the BOM adds
//! to every wire's routed length to give its cut length; the router never
//! reads it.
//!
//! # The network
//!
//! - A **port** is a PORT feature's [`PortRecord`]: a point and a unit
//!   direction. Every port has two SIDES: `A` along the direction, `B` against
//!   it. A wire that enters a port on one side must leave it on the other —
//!   that is the whole routing rule, and what makes a waypoint a pass-through
//!   rather than a junction box.
//! - A **segment** is a SPLINE feature whose first and last anchors attach to
//!   two DIFFERENT ports. The kernel builds the curve from the ports' live
//!   placements (`features/spline.rs`), so the segment's geometry is
//!   authoritative: the PHYSICAL side a segment occupies at each port is read
//!   off the curve's end tangent — `sign(t_out · dir)` at the first port,
//!   `sign(−t_in · dir)` at the last (the wire ARRIVES from the opposite
//!   half-space, so an anchor attached on side A at the end of a spline sits on
//!   the port's side B). The stored attachment side is how the spline was
//!   authored; the tangent is where the wire actually is.
//! - Segment weight = the chain's arc length.
//!
//! # The sided digraph (the retired `sided_ab_graphs` builder, kept exactly)
//!
//! Nodes are `{port}/A` and `{port}/B`. A segment joining `P/X` to `Q/Y` adds
//! two directed edges: `P/X → Q/inv(Y)` and `Q/Y → P/inv(X)`, with
//! `inv(A) = B`. Reading an edge as "leave P through X, arrive at Q on Y, and
//! you are now poised to leave Q through inv(Y)" is what encodes the pass-
//! through rule. A connection routes from `{from}/A` or `{from}/B` to `{to}/A`
//! or `{to}/B` (four Dijkstra queries, the shortest wins). A route that visits
//! the same port twice is refused and the constrained best-first search (state
//! = node + visited-port set) runs instead; if that finds nothing either the
//! connection stays unrouted with the `port-reuse` status. Endpoints must be
//! terminations (a waypoint is a pass-through, never a cable end).
//!
//! # Bundles
//!
//! Every routed connection contributes its `diameter` to each segment it
//! crosses. A segment's bundle diameter is `sqrt(Σ d² / 0.75) · 1.1` (a 75 %
//! packing efficiency and a 10 % safety factor — the established formula), and
//! its solid is a circle of that diameter swept along the spline's exact chain
//! (`sweep_profile_along_chain_with_stations`), named `WireHarness:{spline}`
//! with faces `…:Wall0`, `…:Wall1`, `…:Start`, `…:End`. The bundle solids ride
//! an extra [`FeatureResult`] (`id` [`WIRE_HARNESS_FEATURE_ID`], `type`
//! [`WIRE_HARNESS_FEATURE_TYPE`]) appended to the run's results, so the
//! display pipeline shows them through the standard solid path (rendering
//! requirement R31). A bundle that fails to build reports its error on the
//! bundle row and never halts the run.
//!
//! # Caching
//!
//! The tail caches its last outcome per thread against a fingerprint of the
//! block + the network (port records + segment chains): an unchanged harness
//! replays (`reused`, the same resident handles) instead of re-sweeping —
//! which keeps the main-thread `execute_history` replays the engine makes for
//! consumed-name / assembly sync cheap. A changed fingerprint frees the old
//! bundle handles before building; `clear_history_cache` frees them too.

use std::cell::RefCell;
use std::collections::{BTreeMap, BTreeSet, BinaryHeap, HashMap};

use serde::{Deserialize, Serialize};

use crate::feature_pipeline::features::common;
use crate::feature_pipeline::{Env, FeatureResult, HistoryRequest, PortKind, PortRecord, SceneMap};
use crate::{make_arc, NurbsCurve, Vec3};

/// The id of the appended tail result (and the creator the display pipeline
/// records for every bundle solid). Never a history feature.
pub const WIRE_HARNESS_FEATURE_ID: &str = "WireHarness";
/// The type of the appended tail result.
pub const WIRE_HARNESS_FEATURE_TYPE: &str = "WH";
/// The prefix of every bundle solid name (`WireHarness:{spline id}`).
pub const BUNDLE_SOLID_PREFIX: &str = "WireHarness:";

/// Bundle packing efficiency (the fraction of the bundle cross-section the
/// wires fill).
const PACKING_EFFICIENCY: f64 = 0.75;
/// Bundle safety factor on the diameter.
const SAFETY_FACTOR: f64 = 1.1;
/// The smallest wire / bundle diameter accepted (a zero-diameter wire is a
/// data error, not a thin wire).
const MIN_DIAMETER: f64 = 0.01;
/// Chords per chain piece when the exact arc-length measure refuses a piece —
/// never for the lines and cubic Béziers a spline publishes. Chords read SHORT
/// (32 of them miss 1.0e-4 of a 90° Hermite span), so this is a fallback only.
const LENGTH_SAMPLES: usize = 32;
/// Sweep stations per chain piece for a bundle (a spline span is three pieces).
const STATIONS_PER_PIECE: usize = 12;
/// Zero gates for the end-tangent side test.
const TANGENT_EPS: f64 = 1e-9;

// ===========================================================================
// Sides and attachments (shared with `features/spline.rs`)
// ===========================================================================

/// A port side: `A` along the port direction, `B` against it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
pub enum PortSide {
    A,
    B,
}

impl PortSide {
    /// The opposite side (`inv` in the digraph builder).
    pub fn other(self) -> Self {
        match self {
            PortSide::A => PortSide::B,
            PortSide::B => PortSide::A,
        }
    }

    /// `"A"` / `"B"` (case-insensitive). Anything else is not a side.
    pub fn parse(text: &str) -> Option<Self> {
        match text.trim() {
            "A" | "a" => Some(PortSide::A),
            "B" | "b" => Some(PortSide::B),
            _ => None,
        }
    }

    pub fn letter(self) -> &'static str {
        match self {
            PortSide::A => "A",
            PortSide::B => "B",
        }
    }
}

/// A spline anchor's port attachment: `{ portRef, side }` under the anchor's
/// `attachment` key. `side` defaults to `A` when absent.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Attachment {
    pub port_ref: String,
    pub side: PortSide,
}

impl Attachment {
    /// Parse an anchor's `attachment` value. `None` = not attached (absent,
    /// null, not an object, or an empty `portRef`).
    pub fn parse(value: Option<&serde_json::Value>) -> Option<Self> {
        let object = value?.as_object()?;
        let port_ref = object.get("portRef")?.as_str()?.trim();
        if port_ref.is_empty() {
            return None;
        }
        let side = object
            .get("side")
            .and_then(serde_json::Value::as_str)
            .and_then(PortSide::parse)
            .unwrap_or(PortSide::A);
        Some(Attachment {
            port_ref: port_ref.to_string(),
            side,
        })
    }
}

// ===========================================================================
// The persisted block
// ===========================================================================

/// The document's `wireHarness` block.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct WireHarnessState {
    #[serde(default)]
    pub connections: Vec<WireHarnessConnection>,
    /// Monotonic id counter (`wire-{n}`), never reused.
    #[serde(default, rename = "idCounter")]
    pub id_counter: u64,
    /// Build the bundle solids (default true). Off keeps the routing and the
    /// report but registers no solids.
    #[serde(default = "default_true", rename = "buildBundles")]
    pub build_bundles: bool,
    /// The cut margin, in model units: a set amount added to EVERY wire's
    /// routed length to give the cut length its BOM line carries (MF QTY) —
    /// once per wire, never a percentage and never once per BOM line. It lives
    /// here, in the typed block, because the block is rewritten from this struct
    /// on every harness edit and a key it did not know would be dropped.
    ///
    /// The router never reads it and [`harness_fingerprint`] deliberately does
    /// not hash it: a margin change must not invalidate a route, so the tail
    /// replays its cache and only the BOM's arithmetic moves. Omitted from the
    /// saved block at 0, so a document that never set one saves as before.
    #[serde(default, rename = "cutMargin", skip_serializing_if = "is_zero")]
    pub cut_margin: f64,
}

impl Default for WireHarnessState {
    fn default() -> Self {
        Self {
            connections: Vec::new(),
            id_counter: 0,
            build_bundles: true,
            cut_margin: 0.0,
        }
    }
}

impl WireHarnessState {
    /// Mint the next connection id (`wire-{n}`), bumping the counter.
    pub fn next_id(&mut self) -> String {
        self.id_counter += 1;
        format!("wire-{}", self.id_counter)
    }
}

/// One wire: its id, display name, endpoint PORT ids and diameter.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct WireHarnessConnection {
    pub id: String,
    #[serde(default)]
    pub name: String,
    #[serde(default)]
    pub from: String,
    #[serde(default)]
    pub to: String,
    #[serde(default = "default_diameter")]
    pub diameter: f64,
}

fn default_true() -> bool {
    true
}

fn is_zero(value: &f64) -> bool {
    *value == 0.0
}

fn default_diameter() -> f64 {
    1.0
}

// ===========================================================================
// The report (what the tail hands the app)
// ===========================================================================

/// The routing outcome of one run.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct WireHarnessReport {
    /// Every port the run published, in id order.
    pub endpoints: Vec<WireHarnessEndpoint>,
    /// Every harness segment (a spline attached to two ports at both ends).
    pub segments: Vec<WireHarnessSegment>,
    /// One route per connection, in block order.
    pub routes: Vec<RouteResult>,
    /// One bundle per segment at least one routed connection crosses.
    pub bundles: Vec<WireHarnessBundle>,
    /// Splines that carry attachments but could not become segments, with why
    /// (both ends must attach to two different ports that resolve).
    pub segment_problems: Vec<String>,
}

/// A connection point as an endpoint choice.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct WireHarnessEndpoint {
    /// The point's ADDRESS, which is what `from`/`to` store
    /// (`J1.VCC`, `ACOMP3:J1.VCC`, or a WAYPOINT's feature id).
    pub id: String,
    /// The port group, PART-LOCAL (empty for a waypoint).
    #[serde(default)]
    pub port: String,
    /// The point name, PART-LOCAL — what a symbol pin binds to (a waypoint's
    /// own display name).
    #[serde(default)]
    pub point: String,
    /// The port group's purpose (`pcb` | `wiring` | `piping` | …; empty for a
    /// waypoint).
    #[serde(default)]
    pub purpose: String,
    pub kind: PortKind,
    /// The placed component (ACOMP feature id) carrying this port, when the
    /// port came in with a part rather than from a PORT feature of this
    /// document. The display gates these on the workbench.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub component: Option<String>,
}

/// A network segment: the spline id, its two ports with the PHYSICAL side the
/// wire occupies at each, and its length.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct WireHarnessSegment {
    pub id: String,
    pub first_port: String,
    pub first_side: PortSide,
    pub second_port: String,
    pub second_side: PortSide,
    pub length: f64,
}

/// Why a connection is or is not routed.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum RouteStatus {
    /// Routed through the network.
    Routed,
    /// `from` or `to` is empty or names no port in the scene.
    MissingEndpoint,
    /// `from` or `to` is a waypoint (a pass-through, never a cable end).
    WaypointEndpoint,
    /// `from` and `to` are the same port.
    SameEndpoint,
    /// The network has no segment at all.
    NoSegments,
    /// No sided path joins the two ports.
    NoRoute,
    /// Every path joining the two ports passes through one port twice.
    PortReuse,
}

impl RouteStatus {
    /// The kebab-case word the panel keys its colours on.
    pub fn as_str(self) -> &'static str {
        match self {
            RouteStatus::Routed => "routed",
            RouteStatus::MissingEndpoint => "missing-endpoint",
            RouteStatus::WaypointEndpoint => "waypoint-endpoint",
            RouteStatus::SameEndpoint => "same-endpoint",
            RouteStatus::NoSegments => "no-segments",
            RouteStatus::NoRoute => "no-route",
            RouteStatus::PortReuse => "port-reuse",
        }
    }
}

/// One connection's route.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RouteResult {
    pub connection_id: String,
    pub feasible: bool,
    pub status: RouteStatus,
    /// Human text for the status column (empty when routed).
    pub message: String,
    /// Route length (the sum of the crossed segments' arc lengths).
    pub length: Option<f64>,
    /// The crossed segments (spline ids) in travel order.
    pub segment_ids: Vec<String>,
    /// The sided nodes visited (`{port}/{side}`), start to end. A node names
    /// the side the wire is poised to LEAVE through: the first is the side it
    /// departs the start port on, the last is the opposite of the side it
    /// arrives at the end port on.
    pub node_path: Vec<String>,
    /// The ports visited, start to end.
    pub port_ids: Vec<String>,
}

/// Why a segment's bundle solid is or is not there.
///
/// A ROUTE has had a status since the router existed, because "no route" is an
/// answer about the model rather than a failure of it. A bundle's refusals are
/// the same kind of answer — a spline that turns tighter than the bundle can be
/// pushed round is a thing the user drew, not a thing that went wrong — so it
/// gets a status too, and the panel keys its colour and its word on that rather
/// than on the shape of an error string.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum BundleStatus {
    /// The solid is built and registered under `solid_name`.
    Built,
    /// `buildBundles` is off: the routing is there and the geometry deliberately
    /// is not.
    BundlesOff,
    /// The spline bends TIGHTER than the bundle's own radius, so the swept
    /// surface would pass through itself. `error` names the bend — where it is,
    /// the radius of curvature there, and how far the bundle reaches into it.
    TightBend,
    /// The sweep failed for some other reason; `error` carries it.
    BuildFailed,
}

impl BundleStatus {
    /// The kebab-case word the panel keys its colours on.
    pub fn as_str(self) -> &'static str {
        match self {
            BundleStatus::Built => "built",
            BundleStatus::BundlesOff => "bundles-off",
            BundleStatus::TightBend => "tight-bend",
            BundleStatus::BuildFailed => "build-failed",
        }
    }
}

/// One segment's bundle.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct WireHarnessBundle {
    pub segment_id: String,
    /// The registered solid's name (empty when bundles are off or the build
    /// failed).
    pub solid_name: String,
    pub wire_count: usize,
    pub diameter: f64,
    pub length: f64,
    pub connection_ids: Vec<String>,
    /// Whether the solid is there, and why not when it is not.
    pub status: BundleStatus,
    /// The build failure in full, if the solid could not be swept. Empty
    /// otherwise; the STATUS is what a reader classifies on.
    pub error: String,
}

/// The bundle diameter for a set of wire diameters: `sqrt(Σ d² / 0.75) · 1.1`
/// — 75 % packing efficiency and a 10 % safety factor. Zero for no wires.
pub fn bundle_diameter(diameters: &[f64]) -> f64 {
    let sum_squares: f64 = diameters
        .iter()
        .map(|d| d.max(0.0))
        .map(|d| d * d)
        .sum();
    if sum_squares <= 0.0 {
        return 0.0;
    }
    (sum_squares / PACKING_EFFICIENCY).sqrt() * SAFETY_FACTOR
}

// ===========================================================================
// The tail hook
// ===========================================================================

/// What the tail hands back to the history loop.
pub(crate) struct HarnessOutcome {
    /// The appended result carrying the bundle solids (`None` when there is no
    /// block).
    pub result: Option<FeatureResult>,
    /// The routing report (always present — the panel lists endpoints even
    /// before the first connection exists).
    pub report: Option<WireHarnessReport>,
}

/// Route the request's connections over the scene's ports and spline segments,
/// build the bundle solids, and return them as an extra result plus the report.
/// Runs unconditionally at the tail of every history execution.
pub(crate) fn finish_history_run(
    request: &HistoryRequest,
    scene: &SceneMap,
    _env: &Env,
) -> HarnessOutcome {
    let network = build_network(request, scene);
    let Some(state) = request.wire_harness.as_ref() else {
        // No block: nothing to route and nothing to keep resident.
        clear_cache();
        let report = WireHarnessReport {
            endpoints: network.endpoints(),
            segments: network.segment_rows(),
            segment_problems: network.problems.clone(),
            ..Default::default()
        };
        return HarnessOutcome {
            result: None,
            report: Some(report),
        };
    };

    let fingerprint = harness_fingerprint(state, &network);
    let cached = HARNESS_CACHE.with(|cache| {
        cache.borrow().as_ref().and_then(|entry| {
            (entry.fingerprint == fingerprint).then(|| (entry.result.clone(), entry.report.clone()))
        })
    });
    if let Some((mut result, report)) = cached {
        result.reused = true;
        return HarnessOutcome {
            result: Some(result),
            report: Some(report),
        };
    }
    // The fingerprint moved: the old bundle handles die before the rebuild.
    clear_cache();

    let mut report = WireHarnessReport {
        endpoints: network.endpoints(),
        segments: network.segment_rows(),
        segment_problems: network.problems.clone(),
        ..Default::default()
    };
    let graph = SidedGraph::build(&network);
    for connection in &state.connections {
        report.routes.push(route_connection(&network, &graph, connection));
    }
    let mut result = FeatureResult::empty(WIRE_HARNESS_FEATURE_ID, WIRE_HARNESS_FEATURE_TYPE);
    report.bundles = build_bundles(&network, state, &report.routes, &mut result);

    HARNESS_CACHE.with(|cache| {
        *cache.borrow_mut() = Some(CachedHarness {
            fingerprint,
            result: result.clone(),
            report: report.clone(),
        });
    });
    HarnessOutcome {
        result: Some(result),
        report: Some(report),
    }
}

struct CachedHarness {
    fingerprint: u64,
    result: FeatureResult,
    report: WireHarnessReport,
}

thread_local! {
    static HARNESS_CACHE: RefCell<Option<CachedHarness>> = const { RefCell::new(None) };
}

/// Free the cached bundle solids (a document switch, or a rebuild).
pub fn clear_cache() {
    HARNESS_CACHE.with(|cache| {
        if let Some(entry) = cache.borrow_mut().take() {
            for added in &entry.result.added {
                crate::free_registered_solid(added.handle);
            }
        }
    });
}

/// Hash what the routing reads from the block (each connection's id,
/// endpoints and diameter, in order, plus the bundles switch — a renamed wire
/// or a bumped id counter replays) and everything it reads from the scene:
/// every port record and every segment's identity, sides and exact chain.
/// Bit-exact on the floats — a moved port must rebuild. A port's LABEL is in
/// it too: the report's endpoints (and a waypoint-end message) carry it, and a
/// part's pin rename changes nothing else, so without it the replayed report
/// went on naming the port by its old label.
fn harness_fingerprint(state: &WireHarnessState, network: &Network) -> u64 {
    use std::hash::{Hash, Hasher};
    let mut hasher = std::collections::hash_map::DefaultHasher::new();
    state.build_bundles.hash(&mut hasher);
    state.connections.len().hash(&mut hasher);
    for connection in &state.connections {
        connection.id.hash(&mut hasher);
        connection.from.hash(&mut hasher);
        connection.to.hash(&mut hasher);
        connection.diameter.to_bits().hash(&mut hasher);
    }
    let bits = |v: Vec3, hasher: &mut std::collections::hash_map::DefaultHasher| {
        v.x.to_bits().hash(hasher);
        v.y.to_bits().hash(hasher);
        v.z.to_bits().hash(hasher);
    };
    for (id, port) in &network.ports {
        id.hash(&mut hasher);
        port.local_address().hash(&mut hasher);
        bits(port.point, &mut hasher);
        bits(port.direction, &mut hasher);
        (port.kind == PortKind::Waypoint).hash(&mut hasher);
        port.extension.to_bits().hash(&mut hasher);
    }
    for segment in &network.segments {
        segment.id.hash(&mut hasher);
        segment.first_port.hash(&mut hasher);
        segment.first_side.hash(&mut hasher);
        segment.second_port.hash(&mut hasher);
        segment.second_side.hash(&mut hasher);
        segment.length.to_bits().hash(&mut hasher);
        for curve in &segment.chain {
            curve.degree.hash(&mut hasher);
            for knot in &curve.knots {
                knot.to_bits().hash(&mut hasher);
            }
            for point in &curve.control_points {
                point.x.to_bits().hash(&mut hasher);
                point.y.to_bits().hash(&mut hasher);
                point.z.to_bits().hash(&mut hasher);
                point.w.to_bits().hash(&mut hasher);
            }
        }
    }
    hasher.finish()
}

// ===========================================================================
// The network: ports + segments read off the scene
// ===========================================================================

/// A spline joining two ports.
struct Segment {
    /// The spline feature id.
    id: String,
    first_port: String,
    /// The PHYSICAL side the wire occupies at the first port.
    first_side: PortSide,
    second_port: String,
    second_side: PortSide,
    length: f64,
    chain: Vec<NurbsCurve>,
}

struct Network {
    /// Id order — deterministic endpoint lists and fingerprints.
    ports: BTreeMap<String, PortRecord>,
    /// Port id -> the placed component that carries it (component ports only).
    owners: BTreeMap<String, String>,
    /// Request order.
    segments: Vec<Segment>,
    problems: Vec<String>,
}

impl Network {
    fn endpoints(&self) -> Vec<WireHarnessEndpoint> {
        self.ports
            .iter()
            .map(|(id, port)| WireHarnessEndpoint {
                id: id.clone(),
                port: port.port_name.clone(),
                point: port.point_name.clone(),
                purpose: port.purpose.clone(),
                kind: port.kind,
                component: self.owners.get(id).cloned(),
            })
            .collect()
    }

    fn segment_rows(&self) -> Vec<WireHarnessSegment> {
        self.segments
            .iter()
            .map(|segment| WireHarnessSegment {
                id: segment.id.clone(),
                first_port: segment.first_port.clone(),
                first_side: segment.first_side,
                second_port: segment.second_port.clone(),
                second_side: segment.second_side,
                length: segment.length,
            })
            .collect()
    }

    fn segment(&self, id: &str) -> Option<&Segment> {
        self.segments.iter().find(|segment| segment.id == id)
    }
}

/// Every port in the scene plus every SPLINE feature in the request whose two
/// end anchors attach to two different resolved ports and whose chain the run
/// published (a spline past the rollback has no chain and is skipped).
fn build_network(request: &HistoryRequest, scene: &SceneMap) -> Network {
    let ports: BTreeMap<String, PortRecord> = scene
        .ports
        .iter()
        .map(|(id, port)| (id.clone(), port.clone()))
        .collect();
    let owners: BTreeMap<String, String> = ports
        .keys()
        .filter_map(|id| {
            scene
                .owning_component(id)
                .map(|record| (id.clone(), record.id.clone()))
        })
        .collect();
    let mut segments = Vec::new();
    let mut problems = Vec::new();
    for descriptor in &request.features {
        if !matches!(descriptor.feature_type.as_str(), "SP" | "SPLINE") {
            continue;
        }
        let id = ["id", "featureID"]
            .iter()
            .find_map(|key| descriptor.input_params.get(key).and_then(|v| v.as_str()))
            .unwrap_or("")
            .trim()
            .to_string();
        if id.is_empty() {
            continue;
        }
        let Some(points) = descriptor
            .persistent_data
            .get("spline")
            .and_then(|spline| spline.get("points"))
            .and_then(serde_json::Value::as_array)
        else {
            continue;
        };
        if points.len() < 2 {
            continue;
        }
        let first = Attachment::parse(points.first().and_then(|p| p.get("attachment")));
        let last = Attachment::parse(points.last().and_then(|p| p.get("attachment")));
        let (first, last) = match (first, last) {
            (Some(first), Some(last)) => (first, last),
            // One attached end is authoring in progress — worth a row so the
            // panel can say why the spline is not a segment; none attached is
            // an ordinary spline, not a harness matter.
            (Some(_), None) | (None, Some(_)) => {
                problems.push(format!("{id}: only one end is attached to a port"));
                continue;
            }
            (None, None) => continue,
        };
        if first.port_ref == last.port_ref {
            problems.push(format!("{id}: both ends attach to the same port '{}'", first.port_ref));
            continue;
        }
        let (Some(first_record), Some(last_record)) =
            (ports.get(&first.port_ref), ports.get(&last.port_ref))
        else {
            // The spline itself already reported the missing port as unresolved.
            continue;
        };
        let Some(chain) = scene.resolve_path(&format!("{id}:SplineEdge")).cloned() else {
            continue; // past the rollback, or fully degenerate
        };
        if chain.is_empty() {
            continue;
        }
        let length = chain_length(&chain);
        let first_side = physical_side(first_record, end_tangent(&chain, true), first.side);
        // The wire ARRIVES: it comes from the opposite half-space of its travel
        // direction, so the arriving tangent is negated before the side test.
        let second_side = physical_side(
            last_record,
            end_tangent(&chain, false).scale(-1.0),
            last.side.other(),
        );
        segments.push(Segment {
            id,
            first_port: first.port_ref,
            first_side,
            second_port: last.port_ref,
            second_side,
            length,
            chain,
        });
    }
    Network {
        ports,
        owners,
        segments,
        problems,
    }
}

/// The side of `port` a wire heading along `outward` occupies:
/// `outward · direction ≥ 0` → `A`, else `B`. A zero tangent (a fully
/// degenerate end) falls back to `fallback`.
fn physical_side(port: &PortRecord, outward: Vec3, fallback: PortSide) -> PortSide {
    if outward.length() <= TANGENT_EPS {
        return fallback;
    }
    if outward.dot(port.direction) >= 0.0 {
        PortSide::A
    } else {
        PortSide::B
    }
}

/// The chain's travel tangent at its start (`at_start`) or end: the curve
/// derivative, else the chord to a nearby sample when the derivative is
/// degenerate (a zero Hermite tangent).
fn end_tangent(chain: &[NurbsCurve], at_start: bool) -> Vec3 {
    let curve = if at_start { chain.first() } else { chain.last() };
    let Some(curve) = curve else {
        return Vec3::new(0.0, 0.0, 0.0);
    };
    let Ok([t0, t1]) = curve.domain() else {
        return Vec3::new(0.0, 0.0, 0.0);
    };
    let parameter = if at_start { t0 } else { t1 };
    if let Ok(derivatives) = curve.derivatives(parameter, 1) {
        if let Some(tangent) = derivatives.get(1) {
            if tangent.length() > TANGENT_EPS {
                return *tangent;
            }
        }
    }
    let near = if at_start {
        t0 + (t1 - t0) * 0.05
    } else {
        t1 - (t1 - t0) * 0.05
    };
    match (curve.evaluate(parameter), curve.evaluate(near)) {
        (Ok(at), Ok(nearby)) => {
            if at_start {
                nearby.sub(at)
            } else {
                at.sub(nearby)
            }
        }
        _ => Vec3::new(0.0, 0.0, 0.0),
    }
}

/// Arc length of a chain: the kernel's exact measure
/// ([`crate::curve_arc_length`], Gauss–Legendre paneled at the knots) over each
/// piece. This is the number a wire's cut length is read from, so it must not
/// be a chord sum: chords are a lower bound, and a wire cut short is scrap.
fn chain_length(chain: &[NurbsCurve]) -> f64 {
    let mut total = 0.0;
    for curve in chain {
        let Ok([t0, t1]) = curve.domain() else { continue };
        total += match crate::curve_arc_length(curve, t0, t1) {
            Ok(length) => length,
            Err(_) => chord_length(curve, t0, t1),
        };
    }
    total
}

/// The chord-sum fallback for a piece the exact measure refused.
fn chord_length(curve: &NurbsCurve, t0: f64, t1: f64) -> f64 {
    let Ok(mut previous) = curve.evaluate(t0) else { return 0.0 };
    let mut total = 0.0;
    for sample in 1..=LENGTH_SAMPLES {
        let t = t0 + (t1 - t0) * sample as f64 / LENGTH_SAMPLES as f64;
        if let Ok(point) = curve.evaluate(t) {
            total += point.sub(previous).length();
            previous = point;
        }
    }
    total
}

// ===========================================================================
// The sided digraph + shortest paths
// ===========================================================================

fn node_key(port: &str, side: PortSide) -> String {
    format!("{port}/{}", side.letter())
}

struct Edge {
    to: usize,
    weight: f64,
    /// Index into `Network::segments`.
    segment: usize,
}

struct SidedGraph {
    /// Node key (`{port}/{side}`) → index.
    index: HashMap<String, usize>,
    keys: Vec<String>,
    /// The port each node belongs to (index into `ports`).
    port_of: Vec<usize>,
    ports: Vec<String>,
    edges: Vec<Vec<Edge>>,
}

impl SidedGraph {
    fn build(network: &Network) -> Self {
        let mut graph = SidedGraph {
            index: HashMap::new(),
            keys: Vec::new(),
            port_of: Vec::new(),
            ports: Vec::new(),
            edges: Vec::new(),
        };
        // Every port in the scene gets both sides so an endpoint with no
        // segment still has nodes (and simply reaches nothing).
        for id in network.ports.keys() {
            graph.add_port(id);
        }
        for (segment_index, segment) in network.segments.iter().enumerate() {
            let weight = segment.length.max(1e-9);
            // P/X → Q/inv(Y): leave P through X, arrive at Q on Y, poised to
            // leave Q through the other side.
            let from = graph.node(&segment.first_port, segment.first_side);
            let to = graph.node(&segment.second_port, segment.second_side.other());
            graph.edges[from].push(Edge {
                to,
                weight,
                segment: segment_index,
            });
            // Q/Y → P/inv(X): the same segment travelled the other way.
            let from = graph.node(&segment.second_port, segment.second_side);
            let to = graph.node(&segment.first_port, segment.first_side.other());
            graph.edges[from].push(Edge {
                to,
                weight,
                segment: segment_index,
            });
        }
        graph
    }

    fn add_port(&mut self, id: &str) -> usize {
        if let Some(position) = self.ports.iter().position(|p| p == id) {
            return position;
        }
        let port_index = self.ports.len();
        self.ports.push(id.to_string());
        for side in [PortSide::A, PortSide::B] {
            let key = node_key(id, side);
            self.index.insert(key.clone(), self.keys.len());
            self.keys.push(key);
            self.port_of.push(port_index);
            self.edges.push(Vec::new());
        }
        port_index
    }

    fn node(&mut self, port: &str, side: PortSide) -> usize {
        let key = node_key(port, side);
        if let Some(&index) = self.index.get(&key) {
            return index;
        }
        self.add_port(port);
        self.index[&key]
    }

    fn node_index(&self, port: &str, side: PortSide) -> Option<usize> {
        self.index.get(&node_key(port, side)).copied()
    }
}

/// A found path through the sided graph.
#[derive(Debug, Clone, PartialEq)]
struct SidedPath {
    distance: f64,
    /// Node indices, start to end.
    nodes: Vec<usize>,
    /// Segment indices, one per hop.
    segments: Vec<usize>,
}

/// Ordered f64 for the heap (cost is always finite here).
#[derive(PartialEq)]
struct HeapEntry<T> {
    cost: f64,
    item: T,
}

impl<T: PartialEq> Eq for HeapEntry<T> {}

impl<T: PartialEq> PartialOrd for HeapEntry<T> {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl<T: PartialEq> Ord for HeapEntry<T> {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        // Min-heap on cost: reverse the comparison.
        other
            .cost
            .partial_cmp(&self.cost)
            .unwrap_or(std::cmp::Ordering::Equal)
    }
}

/// Plain Dijkstra from `start` to `end` (node indices).
fn dijkstra(graph: &SidedGraph, start: usize, end: usize) -> Option<SidedPath> {
    let count = graph.keys.len();
    let mut distance = vec![f64::INFINITY; count];
    let mut parent: Vec<Option<(usize, usize)>> = vec![None; count];
    let mut done = vec![false; count];
    let mut heap = BinaryHeap::new();
    distance[start] = 0.0;
    heap.push(HeapEntry {
        cost: 0.0,
        item: start,
    });
    while let Some(HeapEntry { cost, item: node }) = heap.pop() {
        if done[node] {
            continue;
        }
        done[node] = true;
        if node == end {
            break;
        }
        for edge in &graph.edges[node] {
            let next = cost + edge.weight;
            if next < distance[edge.to] {
                distance[edge.to] = next;
                parent[edge.to] = Some((node, edge.segment));
                heap.push(HeapEntry {
                    cost: next,
                    item: edge.to,
                });
            }
        }
    }
    if !distance[end].is_finite() || start == end {
        return None;
    }
    let mut nodes = vec![end];
    let mut segments = Vec::new();
    let mut cursor = end;
    while let Some((previous, segment)) = parent[cursor] {
        segments.push(segment);
        nodes.push(previous);
        cursor = previous;
    }
    nodes.reverse();
    segments.reverse();
    Some(SidedPath {
        distance: distance[end],
        nodes,
        segments,
    })
}

/// Whether a path visits one PORT twice (either side).
fn reuses_a_port(graph: &SidedGraph, path: &SidedPath) -> bool {
    let mut seen = BTreeSet::new();
    path.nodes
        .iter()
        .any(|&node| !seen.insert(graph.port_of[node]))
}

/// Best-first search for the shortest path from `start` to `end` that visits
/// no port twice (state = node + the visited-port set). Exponential in the
/// worst case; harness graphs are tiny, and this only runs when plain
/// Dijkstra's answer reused a port.
fn shortest_non_reusing(graph: &SidedGraph, start: usize, end: usize) -> Option<SidedPath> {
    #[derive(PartialEq)]
    struct State {
        node: usize,
        visited: Vec<bool>,
        nodes: Vec<usize>,
        segments: Vec<usize>,
    }
    let port_count = graph.ports.len();
    let mut best: HashMap<(usize, Vec<bool>), f64> = HashMap::new();
    let mut heap = BinaryHeap::new();
    let mut visited = vec![false; port_count];
    visited[graph.port_of[start]] = true;
    heap.push(HeapEntry {
        cost: 0.0,
        item: State {
            node: start,
            visited,
            nodes: vec![start],
            segments: Vec::new(),
        },
    });
    while let Some(HeapEntry { cost, item: state }) = heap.pop() {
        if state.node == end && !state.segments.is_empty() {
            return Some(SidedPath {
                distance: cost,
                nodes: state.nodes,
                segments: state.segments,
            });
        }
        let key = (state.node, state.visited.clone());
        if best.get(&key).is_some_and(|&known| known < cost - 1e-12) {
            continue;
        }
        for edge in &graph.edges[state.node] {
            let port = graph.port_of[edge.to];
            if state.visited[port] {
                continue;
            }
            let mut visited = state.visited.clone();
            visited[port] = true;
            let next_cost = cost + edge.weight;
            let next_key = (edge.to, visited.clone());
            if best.get(&next_key).is_some_and(|&known| known <= next_cost + 1e-12) {
                continue;
            }
            best.insert(next_key, next_cost);
            let mut nodes = state.nodes.clone();
            nodes.push(edge.to);
            let mut segments = state.segments.clone();
            segments.push(edge.segment);
            heap.push(HeapEntry {
                cost: next_cost,
                item: State {
                    node: edge.to,
                    visited,
                    nodes,
                    segments,
                },
            });
        }
    }
    None
}

/// Route one connection: validate its endpoints, take the shortest of the four
/// sided Dijkstra answers, and fall back to the non-reusing search when that
/// answer passes through a port twice.
fn route_connection(
    network: &Network,
    graph: &SidedGraph,
    connection: &WireHarnessConnection,
) -> RouteResult {
    let unrouted = |status: RouteStatus, message: String| RouteResult {
        connection_id: connection.id.clone(),
        feasible: false,
        status,
        message,
        length: None,
        segment_ids: Vec::new(),
        node_path: Vec::new(),
        port_ids: Vec::new(),
    };
    let from = connection.from.trim();
    let to = connection.to.trim();
    for (label, id) in [("from", from), ("to", to)] {
        if id.is_empty() {
            return unrouted(RouteStatus::MissingEndpoint, format!("no {label} port"));
        }
        let Some(port) = network.ports.get(id) else {
            return unrouted(
                RouteStatus::MissingEndpoint,
                format!("{label} port '{id}' is not in the model"),
            );
        };
        if port.kind == PortKind::Waypoint {
            return unrouted(
                RouteStatus::WaypointEndpoint,
                format!("{label} port '{}' is a waypoint, not a termination", port.local_address()),
            );
        }
    }
    if from == to {
        return unrouted(RouteStatus::SameEndpoint, "from and to are the same port".into());
    }
    if network.segments.is_empty() {
        return unrouted(
            RouteStatus::NoSegments,
            "no harness splines: attach a spline's end anchors to two ports".into(),
        );
    }

    let mut best: Option<SidedPath> = None;
    for start_side in [PortSide::A, PortSide::B] {
        for end_side in [PortSide::A, PortSide::B] {
            let (Some(start), Some(end)) = (
                graph.node_index(from, start_side),
                graph.node_index(to, end_side),
            ) else {
                continue;
            };
            if let Some(path) = dijkstra(graph, start, end) {
                if best.as_ref().is_none_or(|b| path.distance < b.distance) {
                    best = Some(path);
                }
            }
        }
    }
    let Some(mut path) = best else {
        return unrouted(
            RouteStatus::NoRoute,
            "no sided path joins the two ports (check which side each spline leaves its ports on)".into(),
        );
    };
    if reuses_a_port(graph, &path) {
        let mut alternative: Option<SidedPath> = None;
        for start_side in [PortSide::A, PortSide::B] {
            for end_side in [PortSide::A, PortSide::B] {
                let (Some(start), Some(end)) = (
                    graph.node_index(from, start_side),
                    graph.node_index(to, end_side),
                ) else {
                    continue;
                };
                if let Some(found) = shortest_non_reusing(graph, start, end) {
                    if alternative.as_ref().is_none_or(|b| found.distance < b.distance) {
                        alternative = Some(found);
                    }
                }
            }
        }
        match alternative {
            Some(found) => path = found,
            None => {
                return unrouted(
                    RouteStatus::PortReuse,
                    "every path passes through the same port twice".into(),
                )
            }
        }
    }
    RouteResult {
        connection_id: connection.id.clone(),
        feasible: true,
        status: RouteStatus::Routed,
        message: String::new(),
        length: Some(path.distance),
        segment_ids: path
            .segments
            .iter()
            .map(|&index| network.segments[index].id.clone())
            .collect(),
        node_path: path.nodes.iter().map(|&node| graph.keys[node].clone()).collect(),
        port_ids: path
            .nodes
            .iter()
            .map(|&node| graph.ports[graph.port_of[node]].clone())
            .collect(),
    }
}

// ===========================================================================
// Bundles
// ===========================================================================

/// Group the routed connections per segment, size each bundle, and (when the
/// block asks for it) sweep a circle of that diameter along the segment's
/// chain into a registered solid on `result`.
fn build_bundles(
    network: &Network,
    state: &WireHarnessState,
    routes: &[RouteResult],
    result: &mut FeatureResult,
) -> Vec<WireHarnessBundle> {
    // Segment id → (wire diameters, connection ids), in first-use order.
    let mut usage: Vec<(String, Vec<f64>, Vec<String>)> = Vec::new();
    for route in routes.iter().filter(|route| route.feasible) {
        let Some(connection) = state
            .connections
            .iter()
            .find(|connection| connection.id == route.connection_id)
        else {
            continue;
        };
        let diameter = connection.diameter.max(MIN_DIAMETER);
        for segment_id in &route.segment_ids {
            let entry = match usage.iter_mut().find(|(id, _, _)| id == segment_id) {
                Some(entry) => entry,
                None => {
                    usage.push((segment_id.clone(), Vec::new(), Vec::new()));
                    usage.last_mut().expect("just pushed")
                }
            };
            entry.1.push(diameter);
            if !entry.2.contains(&connection.id) {
                entry.2.push(connection.id.clone());
            }
        }
    }

    let mut bundles = Vec::with_capacity(usage.len());
    for (segment_id, diameters, connection_ids) in usage {
        let Some(segment) = network.segment(&segment_id) else {
            continue;
        };
        let diameter = bundle_diameter(&diameters).max(MIN_DIAMETER);
        let mut bundle = WireHarnessBundle {
            segment_id: segment_id.clone(),
            solid_name: String::new(),
            wire_count: diameters.len(),
            diameter,
            length: segment.length,
            connection_ids,
            status: BundleStatus::BundlesOff,
            error: String::new(),
        };
        if state.build_bundles {
            let name = format!("{BUNDLE_SOLID_PREFIX}{segment_id}");
            match sweep_bundle(segment, diameter * 0.5, &name) {
                Ok(solid) => {
                    result.added.push(common::register_added(solid, &name));
                    bundle.solid_name = name;
                    bundle.status = BundleStatus::Built;
                }
                Err(error) => {
                    // Classified on the refusal's own CLASS, minted where the
                    // sweep decides the bend is too tight: the sweep owns the
                    // sentence, this owns the status, and a rewording of the
                    // one cannot move the other.
                    bundle.status = if crate::is_sweep_tight_bend(&error) {
                        BundleStatus::TightBend
                    } else {
                        BundleStatus::BuildFailed
                    };
                    bundle.error = error.message;
                }
            }
        }
        bundles.push(bundle);
    }
    bundles
}

/// Sweep a circle of `radius` along the segment's chain: two half-arcs in the
/// plane square to the chain's start tangent (a closed profile needs at least
/// two curves), the station budget scaled with the chain's piece count.
fn sweep_bundle(
    segment: &Segment,
    radius: f64,
    name: &str,
) -> Result<crate::BrepSolid, crate::KernelRefusal> {
    use crate::{KernelRefusal, KernelStage, OrRefuse};
    let start = segment.chain[0]
        .domain()
        .and_then(|[t0, _]| segment.chain[0].evaluate(t0))
        .or_refuse(KernelStage::Collect, "evaluate")?;
    let tangent = end_tangent(&segment.chain, true).normalized().map_err(|_| {
        KernelRefusal::input(KernelStage::Collect, "zero_tangent", "the chain starts with a zero tangent")
    })?;
    let x_axis = tangent.perpendicular().or_refuse(KernelStage::Collect, "perpendicular")?;
    let y_axis = tangent.cross(x_axis).normalized().or_refuse(KernelStage::Collect, "normalized")?;
    let profile = vec![
        make_arc(start, x_axis, y_axis, radius, 0.0, std::f64::consts::PI)
            .or_refuse(KernelStage::Fragment, "make_arc")?,
        make_arc(start, x_axis, y_axis, radius, std::f64::consts::PI, std::f64::consts::TAU)
            .or_refuse(KernelStage::Fragment, "make_arc")?,
    ];
    let names: Vec<String> = (0..segment.chain.len())
        .map(|index| format!("{}:piece{index}", segment.id))
        .collect();
    let stations = (segment.chain.len() * STATIONS_PER_PIECE).clamp(32, 1024);
    // The chain is classified here, where it is assembled: a harness span is a
    // spline cut into pieces, so it is open and tangent-continuous by
    // construction, and the classification is what lets the builder SAY so when a
    // zero-length anchor extension leaves a corner in it anyway.
    let path = crate::SweepPath::new(segment.chain.clone(), names)?;
    let mut solid = crate::sweep_profile_along_chain_with_stations(
        &profile,
        &path,
        stations,
        "a harness segment must be tangent-continuous (a spline always is; a zero extension at an anchor can leave a corner)",
    )?;
    // Face names: the two walls (profile-curve order), then START and END.
    let faces = &mut solid
        .shells
        .get_mut(0)
        .ok_or_else(|| KernelRefusal::internal(KernelStage::Sew, "bundle_shell", "the sweep produced no shell"))?
        .faces;
    let expected = ["Wall0", "Wall1", "Start", "End"];
    if faces.len() != expected.len() {
        return Err(KernelRefusal::internal(
            KernelStage::Sew,
            "bundle_faces",
            format!("the sweep produced {} faces, expected {}", faces.len(), expected.len()),
        ));
    }
    for (face, suffix) in faces.iter_mut().zip(expected) {
        face.name = Some(format!("{name}:{suffix}"));
    }
    Ok(solid)
}

