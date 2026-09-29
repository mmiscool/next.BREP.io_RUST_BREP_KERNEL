//! Copper zones: a polygon on one copper layer, given to one net, and FILLED with
//! copper wherever the board's rules allow it — a ground pour.
//!
//! # The fill rule
//!
//! The fill of a zone on layer `L` for net `N` is the zone's polygon
//!
//! * inside the board outline by at least [`DesignRules::edge_clearance`];
//! * minus a clearance round every piece of copper on `L` that is NOT on `N` —
//!   pads, tracks and vias of other nets, and copper that carries no net at all
//!   (an unrouted stub, a mechanical pad). The clearance is the larger of `N`'s net
//!   class's and that copper's ([`DesignRules::clearance_between`]; netless copper
//!   is Default);
//! * minus the whole outline of every EARLIER zone on `L` of another net, grown by
//!   the larger of the two nets' class clearances: zones are filled in the order they were drawn and the first
//!   one keeps the ground they share (KiCad's priority, fixed to drawing order);
//! * with same-net copper joined as follows — a surface-mount pad, a track and a
//!   via of `N` are SOLID: the fill runs over them. A through-hole pad of `N` gets
//!   a THERMAL RELIEF: a gap of [`Zone::thermal_gap`] (at least `N`'s class clearance) all
//!   round it, bridged by up to four straight spokes, [`Zone::spoke_width`] wide,
//!   left, right, up and down. A spoke is laid only where the fill is there to
//!   meet it, so [`Thermal::spokes`] counts spokes that connect;
//! * opened by [`Zone::min_width`]: every part of the fill narrower than that is
//!   dropped (erode by half the width, grow back by half), before the spokes go
//!   in so a spoke is never eroded away;
//! * with its ISLANDS removed: a separate piece of fill that touches no copper of
//!   `N` is dropped.
//!
//! A via with no net of its own whose centre is inside the zone's polygon is taken
//! as a STITCHING via of the zone's net rather than as copper to keep clear of —
//! a via dropped into a ground pour is there to join it to the other side.
//! Which net a netless via stitches is decided ONCE for the board, per copper
//! island, before any zone is filled: the nets of every zone that holds one of
//! the island's vias. One net, and the island is that net's. Two or more — a
//! via where a GND pour on one side and a VCC pour on the other overlap — and it
//! is stitched to NONE: each fill keeps clear of it, as of any other copper, and
//! the fill report and the design rule check both name it
//! ([`Board::contested_vias`]). Stitched to both, the plated barrel would join
//! the two fills and the Gerbers would carry the short.
//!
//! # How it is computed
//!
//! On a square grid of [`PITCH`] (finer when the clearance is small), each sample
//! holds how far it is INSIDE the fill region: the least of its distance inside
//! the zone, inside the outline less the edge clearance, and its distance from
//! each piece of other-net copper less the clearance. The fill's boundary is that
//! function's zero contour, traced by marching squares with the crossing on each
//! grid edge interpolated — so an edge round a pad is a smooth curve and not a
//! staircase — and every ring is simplified to within [`SIMPLIFY`]. The contour is
//! taken at [`Self`]'s margin rather than at zero, so what interpolation and
//! simplification can cost is paid out of the fill and never out of the
//! clearance: a fill measured from its Gerber is never nearer other copper than
//! the rule.
//!
//! # When it is filled
//!
//! On demand ([`Board::fill_zones`], the board's Fill zones action) and by the
//! board view's design rule check, which refills before it checks. NOT on every
//! edit: the fill is the costliest thing on a board after the autorouter, and a
//! drag is sixty edits a second. What was filled is STORED in the zone, so the
//! Gerbers, the 3D view and the STEP export all draw the copper the user saw; the
//! fabrication export refills a copy first, so a stale fill is never what is
//! sent to be made.
use super::{Board, CopperItem, CopperRef, Connectivity, Shape, polygon_contains, point_segment_distance};
use crate::{Document, Netlist, Point, Uuid};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet, HashMap};

/// A thermal relief's default gap round a through-hole pad: KiCad's 0.5 mm.
pub const DEFAULT_THERMAL_GAP: i32 = 500;
/// A thermal spoke's default width: KiCad's 0.5 mm.
pub const DEFAULT_SPOKE_WIDTH: i32 = 500;
/// The narrowest fill kept by default: KiCad's 0.25 mm.
pub const DEFAULT_MIN_WIDTH: i32 = 250;
/// The fill grid's pitch in micrometres, unless the clearance asks for a finer one.
pub const PITCH: i32 = 50;
/// How far, in micrometres, a simplified fill edge may stray from the traced one.
pub const SIMPLIFY: f64 = 2.;
/// The most grid samples one zone's fill takes; a bigger zone gets a coarser grid.
const MAX_SAMPLES: usize = 12_000_000;

fn default_thermal_gap() -> i32 {
    DEFAULT_THERMAL_GAP
}
fn default_spoke_width() -> i32 {
    DEFAULT_SPOKE_WIDTH
}
fn default_min_width() -> i32 {
    DEFAULT_MIN_WIDTH
}

/// A copper zone: an outline on one copper layer, the net it pours, and the copper
/// its last fill made.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Zone {
    pub id: Uuid,
    /// The net the zone pours, by name.
    pub net: String,
    pub layer: u8,
    /// Closed outline polygon, in board micrometres.
    pub outline: Vec<Point>,
    /// Gap round a through-hole pad of the zone's net, bridged by spokes.
    #[serde(default = "default_thermal_gap")]
    pub thermal_gap: i32,
    #[serde(default = "default_spoke_width")]
    pub spoke_width: i32,
    /// Fill narrower than this is dropped.
    #[serde(default = "default_min_width")]
    pub min_width: i32,
    /// The copper the last fill made. Empty until the zone is filled.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub fill: Vec<FillPiece>,
}
impl Zone {
    pub fn new(net: &str, layer: u8, outline: Vec<Point>) -> Self {
        Self {
            id: Uuid::new_v4(),
            net: net.to_owned(),
            layer,
            outline,
            thermal_gap: DEFAULT_THERMAL_GAP,
            spoke_width: DEFAULT_SPOKE_WIDTH,
            min_width: DEFAULT_MIN_WIDTH,
            fill: vec![],
        }
    }
    /// Filled copper area in square micrometres.
    pub fn fill_area(&self) -> f64 {
        // `+ 0.`: an empty float sum is -0.0, which read "-0.0 mm²".
        self.fill.iter().map(FillPiece::area).sum::<f64>() + 0.
    }
    pub(super) fn validate(&self, layer_count: u8) -> Result<(), String> {
        let valid = |p: &Point| p.x.abs_diff(0) <= 100_000_000 && p.y.abs_diff(0) <= 100_000_000;
        if self.layer >= layer_count {
            return Err("A zone is on a copper layer the board does not have".into());
        }
        if self.net.trim().is_empty() {
            return Err("A zone needs a net".into());
        }
        if self.outline.len() < 3 || !self.outline.iter().all(valid) || ring_area(&self.outline) == 0. {
            return Err("A zone's outline needs at least three points enclosing an area".into());
        }
        let size = |v: i32| (1..=100_000_000).contains(&v);
        if !size(self.thermal_gap) || !size(self.spoke_width) || !size(self.min_width) {
            return Err("A zone's thermal gap, spoke width and minimum width must be positive".into());
        }
        let rings = self.fill.iter().flat_map(|p| std::iter::once(&p.outer).chain(&p.holes));
        for ring in rings {
            if ring.len() < 3 || !ring.iter().all(valid) {
                return Err("A zone's fill has a ring of fewer than three valid points".into());
            }
        }
        Ok(())
    }
}

/// One connected piece of a zone's fill: its outer ring and the holes in it.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct FillPiece {
    pub outer: Vec<Point>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub holes: Vec<Vec<Point>>,
}
impl FillPiece {
    /// Whether `p` is copper of this piece: inside the outer ring and in no hole.
    pub fn contains(&self, p: Point) -> bool {
        polygon_contains(&self.outer, p) && !self.holes.iter().any(|h| polygon_contains(h, p))
    }
    /// Every edge of every ring.
    pub fn edges(&self) -> impl Iterator<Item = (Point, Point)> + '_ {
        std::iter::once(&self.outer)
            .chain(&self.holes)
            .flat_map(|ring| (0..ring.len()).map(move |i| (ring[i], ring[(i + 1) % ring.len()])))
    }
    /// Area in square micrometres.
    pub fn area(&self) -> f64 {
        ring_area(&self.outer).abs() - self.holes.iter().map(|h| ring_area(h).abs()).sum::<f64>()
    }
    pub fn bounds(&self) -> (Point, Point) {
        super::bounds_of(&self.outer).unwrap_or_default()
    }
    /// The gap between this piece and a copper shape; zero when they touch or overlap.
    pub fn distance(&self, shape: &Shape) -> f64 {
        if self.contains(shape.center()) {
            return 0.;
        }
        self.edges()
            .map(|(a, b)| Shape::segment(a, b, 0).distance(shape))
            .fold(f64::INFINITY, f64::min)
    }
    /// Whether a copper shape touches or overlaps this piece, to within 0.5 µm:
    /// [`Self::distance`] `<= 0.5`, without measuring the edges that are nowhere
    /// near it. The board view asks this for every item on the zone's layer on
    /// every frame, so it pays for the edges beside the shape and no others.
    pub fn touches(&self, shape: &Shape) -> bool {
        let (lo, hi) = shape.bounds();
        let near = |a: Point, b: Point| {
            a.x.min(b.x) <= hi.x + 1 && a.x.max(b.x) >= lo.x - 1 && a.y.min(b.y) <= hi.y + 1 && a.y.max(b.y) >= lo.y - 1
        };
        self.edges()
            .any(|(a, b)| near(a, b) && Shape::segment(a, b, 0).distance(shape) <= 0.5)
            || self.contains(shape.center())
    }
    /// The gap between two pieces; zero when they touch or overlap.
    pub fn distance_to_piece(&self, other: &FillPiece) -> f64 {
        if other.outer.first().is_some_and(|p| self.contains(*p))
            || self.outer.first().is_some_and(|p| other.contains(*p))
        {
            return 0.;
        }
        let (min, max) = other.bounds();
        let mut best = f64::INFINITY;
        for (a, b) in self.edges() {
            let edge = Shape::segment(a, b, 0);
            let (e0, e1) = edge.bounds();
            let far = f64::from((min.x - e1.x).max(e0.x - max.x).max(min.y - e1.y).max(e0.y - max.y));
            if far >= best {
                continue;
            }
            for (c, d) in other.edges() {
                best = best.min(edge.distance(&Shape::segment(c, d, 0)));
            }
        }
        best
    }
}

/// The signed area of a ring in square micrometres, positive when it
/// turns from +x towards +y.
pub fn ring_area(ring: &[Point]) -> f64 {
    let mut sum = 0i128;
    for i in 0..ring.len() {
        let (a, b) = (ring[i], ring[(i + 1) % ring.len()]);
        sum += i128::from(a.x) * i128::from(b.y) - i128::from(b.x) * i128::from(a.y);
    }
    sum as f64 / 2.
}

/// A through-hole pad of a zone's net and the spokes that join it to the fill.
#[derive(Clone, Debug, PartialEq)]
pub struct Thermal {
    pub owner: CopperRef,
    pub at: Point,
    /// Spokes laid, 0 to 4. Zero means the zone does not reach the pad.
    pub spokes: u8,
}

/// What one zone's fill did.
#[derive(Clone, Debug, PartialEq)]
pub struct ZoneReport {
    pub zone: Uuid,
    pub pieces: usize,
    /// Filled area in square micrometres.
    pub area: f64,
    /// Pieces dropped because they touched no copper of the zone's net.
    pub islands_removed: usize,
    pub thermals: Vec<Thermal>,
    /// The grid pitch the fill was computed on, in micrometres.
    pub pitch: i32,
    /// Why the zone has no fill at all, when it has none.
    pub empty_because: Option<String>,
    /// Netless vias inside this zone that it did NOT stitch, because a zone of
    /// another net holds them too: each via's centre and the other nets.
    pub unstitched: Vec<(Point, Vec<String>)>,
}

/// A via with no net that zones of more than one net hold, and which is therefore
/// stitched to none of them.
#[derive(Clone, Debug, PartialEq)]
pub struct ContestedVia {
    /// Index into [`Board::vias`].
    pub via: usize,
    pub at: Point,
    /// Every net whose zones hold a via of this via's copper island, in order.
    pub nets: Vec<String>,
    /// The zones that hold THIS via, by index into [`Board::zones`].
    pub zones: Vec<usize>,
}

/// Which net each netless copper island is stitched to, decided once for the
/// board: the island's vias, the zones whose polygon holds one of them on a
/// layer the via reaches, and those zones' nets. `conn` is the connectivity
/// WITHOUT any fill.
struct Stitching {
    /// The island's net, where exactly one net's zones hold its vias.
    net: HashMap<usize, String>,
    /// Every net claiming each island that more than one net claims.
    contested: BTreeMap<usize, BTreeSet<String>>,
}
fn holds(zone: &Zone, item: &CopperItem) -> bool {
    item.layers.0 <= zone.layer && zone.layer <= item.layers.1 && polygon_contains(&zone.outline, item.shape.center())
}
fn stitching(board: &Board, conn: &Connectivity) -> Stitching {
    let mut claims: BTreeMap<usize, BTreeSet<String>> = BTreeMap::new();
    for (i, item) in conn.items.iter().enumerate() {
        let island = conn.islands[i];
        if !matches!(item.owner, CopperRef::Via(_)) || !conn.island_nets[island].is_empty() {
            continue;
        }
        for zone in board.zones.iter().filter(|z| holds(z, item)) {
            claims.entry(island).or_default().insert(zone.net.clone());
        }
    }
    let mut out = Stitching { net: HashMap::new(), contested: BTreeMap::new() };
    for (island, nets) in claims {
        if nets.len() == 1 {
            out.net.insert(island, nets.into_iter().next().unwrap());
        } else {
            out.contested.insert(island, nets);
        }
    }
    out
}

impl Document {
    /// Fill every zone of the board against the schematic's nets.
    pub fn fill_zones(&mut self) -> Vec<ZoneReport> {
        let netlist = self.netlist();
        self.board.fill_zones(&netlist)
    }
}

impl Board {
    /// Refill every zone, in the order they were drawn, and say what each fill did.
    pub fn fill_zones(&mut self, netlist: &Netlist) -> Vec<ZoneReport> {
        let conn = self.connectivity_of(netlist, false);
        let stitch = stitching(self, &conn);
        let mut reports = Vec::with_capacity(self.zones.len());
        for index in 0..self.zones.len() {
            let (fill, report) = fill_zone(self, &conn, &stitch, index);
            self.zones[index].fill = fill;
            reports.push(report);
        }
        reports
    }
    /// Whether any zone's fill differs from what a fill now would make.
    pub fn zones_stale(&self, netlist: &Netlist) -> bool {
        if self.zones.is_empty() {
            return false;
        }
        let mut fresh = self.clone();
        fresh.fill_zones(netlist);
        fresh.zones != self.zones
    }
    /// The vias with no net that zones of two or more nets hold, which the fill
    /// therefore stitches to none of them, in the order of [`Board::vias`].
    pub fn contested_vias(&self, netlist: &Netlist) -> Vec<ContestedVia> {
        if self.zones.is_empty() {
            return vec![];
        }
        let conn = self.connectivity_of(netlist, false);
        let stitch = stitching(self, &conn);
        let mut out = vec![];
        for (i, item) in conn.items.iter().enumerate() {
            let CopperRef::Via(via) = item.owner else { continue };
            let Some(nets) = stitch.contested.get(&conn.islands[i]) else { continue };
            let zones: Vec<usize> = (0..self.zones.len()).filter(|&z| holds(&self.zones[z], item)).collect();
            if !zones.is_empty() {
                out.push(ContestedVia { via, at: item.shape.center(), nets: nets.iter().cloned().collect(), zones });
            }
        }
        out
    }
    /// A design rule finding for each contested via ([`Self::contested_vias`]),
    /// naming it by where it is and the zones that hold it.
    pub(super) fn stitching_findings(&self, netlist: &Netlist) -> Vec<super::Violation> {
        self.contested_vias(netlist)
            .into_iter()
            .map(|c| {
                let zones: Vec<String> = c.zones.iter().map(|&z| describe(&self.zones[z], self.layer_count)).collect();
                let own: BTreeSet<&str> = c.zones.iter().map(|&z| self.zones[z].net.as_str()).collect();
                let others: Vec<&str> = c.nets.iter().map(String::as_str).filter(|n| !own.contains(n)).collect();
                // Held by zones of both nets itself, or by one net's zone and
                // joined by copper to a via another net's zone holds.
                let place = if others.is_empty() {
                    format!("is in both the {}", zones.join(" and the "))
                } else {
                    format!(
                        "is in the {}, and copper joins it to a via in a {} zone",
                        zones.join(" and the "),
                        others.join(" and ")
                    )
                };
                super::Violation {
                    kind: super::ViolationKind::Stitching,
                    message: format!(
                        "Via at {:.2}, {:.2} mm has no net and {place}: it is stitched to none of them, \
                         because that would short {}. Give it a net, or keep it in one net's zones",
                        f64::from(c.at.x) / 1000.,
                        f64::from(c.at.y) / 1000.,
                        c.nets.join(" to ")
                    ),
                    at: c.at,
                }
            })
            .collect()
    }
}

/// What the field at a sample is made of: the copper to keep clear of, and the
/// pads to relieve.
enum Obstacle {
    /// Other-net copper: keep `clearance` from the shape.
    Keep(Shape, f64),
    /// Another zone's outline, taken whole: keep `clearance` from it.
    Zone(Vec<Point>, f64),
}

/// Signed distance from `p` to a closed polygon: positive inside.
fn polygon_depth(polygon: &[Point], p: Point) -> f64 {
    let n = polygon.len();
    let d = (0..n)
        .map(|i| point_segment_distance(p, polygon[i], polygon[(i + 1) % n]))
        .fold(f64::INFINITY, f64::min);
    if polygon_contains(polygon, p) { d } else { -d }
}

/// A sampled scalar field over a rectangle of the board.
struct Grid {
    x0: i32,
    y0: i32,
    h: i32,
    nx: usize,
    ny: usize,
    v: Vec<f32>,
}
impl Grid {
    fn at(&self, i: usize, j: usize) -> Point {
        Point::new(self.x0 + i as i32 * self.h, self.y0 + j as i32 * self.h)
    }
    fn get(&self, i: usize, j: usize) -> f32 {
        self.v[j * self.nx + i]
    }
    /// The field at any point, bilinear between samples; `-inf` off the grid.
    fn sample(&self, p: Point) -> f32 {
        let fx = f64::from(p.x - self.x0) / f64::from(self.h);
        let fy = f64::from(p.y - self.y0) / f64::from(self.h);
        if fx < 0. || fy < 0. || fx >= (self.nx - 1) as f64 || fy >= (self.ny - 1) as f64 {
            return f32::NEG_INFINITY;
        }
        let (i, j) = (fx as usize, fy as usize);
        let (tx, ty) = ((fx - i as f64) as f32, (fy - j as f64) as f32);
        let top = self.get(i, j) * (1. - tx) + self.get(i + 1, j) * tx;
        let bottom = self.get(i, j + 1) * (1. - tx) + self.get(i + 1, j + 1) * tx;
        top * (1. - ty) + bottom * ty
    }
}

/// Buckets of obstacles by square cell, so a sample asks only those near it.
struct Buckets {
    x0: i32,
    y0: i32,
    size: i32,
    nx: usize,
    ny: usize,
    cells: Vec<Vec<u32>>,
}
impl Buckets {
    fn new(min: Point, max: Point, size: i32) -> Self {
        let nx = ((max.x - min.x) / size + 1).max(1) as usize;
        let ny = ((max.y - min.y) / size + 1).max(1) as usize;
        Self { x0: min.x, y0: min.y, size, nx, ny, cells: vec![vec![]; nx * ny] }
    }
    fn cell(&self, x: i32, y: i32) -> (usize, usize) {
        let i = ((x - self.x0).div_euclid(self.size)).clamp(0, self.nx as i32 - 1) as usize;
        let j = ((y - self.y0).div_euclid(self.size)).clamp(0, self.ny as i32 - 1) as usize;
        (i, j)
    }
    fn insert(&mut self, index: u32, min: Point, max: Point) {
        let (i0, j0) = self.cell(min.x, min.y);
        let (i1, j1) = self.cell(max.x, max.y);
        for j in j0..=j1 {
            for i in i0..=i1 {
                self.cells[j * self.nx + i].push(index);
            }
        }
    }
    fn near(&self, p: Point) -> &[u32] {
        let (i, j) = self.cell(p.x, p.y);
        &self.cells[j * self.nx + i]
    }
}

/// The net each item's copper island carries, and which vias count as the zone's.
fn item_net<'a>(conn: &'a Connectivity, i: usize) -> Option<&'a str> {
    conn.island_net(conn.islands[i])
}

/// Fill zone `index` of `board`. `conn` is the board's connectivity WITHOUT any
/// zone's fill, so a stale fill never decides which net a track is on.
fn fill_zone(board: &Board, conn: &Connectivity, stitch: &Stitching, index: usize) -> (Vec<FillPiece>, ZoneReport) {
    let zone = &board.zones[index];
    let mut report = ZoneReport {
        zone: zone.id,
        pieces: 0,
        area: 0.,
        islands_removed: 0,
        thermals: vec![],
        pitch: PITCH,
        empty_because: None,
        unstitched: vec![],
    };
    let rules = &board.rules;
    // Each piece of copper is kept clear by the larger of its net's class
    // clearance and the zone's ([`DesignRules::clearance_between`]); the zone's own
    // class clearance is the least any obstacle is kept clear by.
    let own = rules.class_of(Some(zone.net.as_str())).clearance.max(0);
    let clearance_to = |nets: &BTreeSet<String>| {
        f64::from(rules.clearance_between_nets([zone.net.as_str()], nets.iter().map(String::as_str)).0)
    };
    let gap = f64::from(zone.thermal_gap.max(own));
    let half_width = f64::from(zone.min_width) / 2.;
    let spoke_half = f64::from(zone.spoke_width) / 2.;
    let layer = zone.layer;
    let on_layer = |item: &CopperItem| item.layers.0 <= layer && layer <= item.layers.1;

    // The rectangle to sample: the zone's bounds within the outline's.
    let (Some((zmin, zmax)), (omin, omax)) = (super::bounds_of(&zone.outline), board.outline_bounds()) else {
        report.empty_because = Some("the zone has no outline".into());
        return (vec![], report);
    };
    let min = Point::new(zmin.x.max(omin.x), zmin.y.max(omin.y));
    let max = Point::new(zmax.x.min(omax.x), zmax.y.min(omax.y));
    if min.x >= max.x || min.y >= max.y {
        report.empty_because = Some("the zone is outside the board".into());
        return (vec![], report);
    }
    // A pitch fine enough that a clearance spans two samples, coarse enough that
    // the grid stays within MAX_SAMPLES.
    let mut h = PITCH.min((own / 2).max(10));
    let area = |h: i32| (((max.x - min.x) / h + 3) as usize) * (((max.y - min.y) / h + 3) as usize);
    while area(h) > MAX_SAMPLES {
        h *= 2;
    }
    report.pitch = h;
    // What interpolation, the simplification and rounding to whole micrometres can
    // cost at worst, taken out of the fill: the chord across a cell of a curve of
    // radius the clearance sags by (√2 h)² / 8r, and the rest is fixed.
    let radius = f64::from(own.max(h));
    let margin = (2. * f64::from(h * h) / (8. * radius) + SIMPLIFY + 1. + 1.) as f32;

    // One ring of samples beyond the rectangle on every side, all outside, so every
    // contour closes.
    let x0 = min.x - h;
    let y0 = min.y - h;
    let nx = ((max.x - min.x) / h + 3) as usize;
    let ny = ((max.y - min.y) / h + 3) as usize;

    // The copper: what to keep clear of, what to relieve, what is solid.
    let mut obstacles: Vec<Obstacle> = vec![];
    let mut thermal_pads: Vec<(CopperRef, Shape)> = vec![];
    let mut same_net: Vec<usize> = vec![];
    for (i, item) in conn.items.iter().enumerate() {
        if !on_layer(item) {
            continue;
        }
        let net = item_net(conn, i);
        // A netless via in the zone joins it only when its island was given the
        // zone's net for the whole board ([`stitching`]); one another net's zone
        // also holds is kept clear of, and said.
        let island = conn.islands[i];
        let held = matches!(item.owner, CopperRef::Via(_)) && holds(zone, item);
        let stitched = held && stitch.net.get(&island) == Some(&zone.net);
        if held && let Some(nets) = stitch.contested.get(&island) {
            let others = nets.iter().filter(|n| **n != zone.net).cloned().collect();
            report.unstitched.push((item.shape.center(), others));
        }
        if net == Some(zone.net.as_str()) || stitched {
            same_net.push(i);
            if let CopperRef::Pad { placement, pad } = item.owner
                && board.placements[placement].footprint.pads[pad].drill.is_some()
            {
                thermal_pads.push((item.owner, item.shape));
            }
        } else {
            obstacles.push(Obstacle::Keep(item.shape, clearance_to(&conn.island_nets[island])));
        }
    }
    for earlier in &board.zones[..index] {
        if earlier.layer == layer && earlier.net != zone.net && earlier.outline.len() >= 3 {
            let clearance = f64::from(rules.clearance_between(Some(zone.net.as_str()), Some(earlier.net.as_str())).0);
            obstacles.push(Obstacle::Zone(earlier.outline.clone(), clearance));
        }
    }
    let clearance = obstacles
        .iter()
        .map(|o| match o {
            Obstacle::Keep(_, c) | Obstacle::Zone(_, c) => *c,
        })
        .fold(f64::from(own), f64::max);
    // Far from every obstacle the field only needs to be known up to a cap: past
    // it, nothing the fill decides changes.
    let cap = (half_width + 4. * f64::from(h)) as f32;
    let reach = (clearance.max(gap) + f64::from(cap)) as i32 + h;
    let mut buckets = Buckets::new(Point::new(x0, y0), Point::new(x0 + nx as i32 * h, y0 + ny as i32 * h), 1000);
    for (k, obstacle) in obstacles.iter().enumerate() {
        let (lo, hi) = match obstacle {
            Obstacle::Keep(shape, _) => shape.bounds(),
            Obstacle::Zone(outline, _) => super::bounds_of(outline).unwrap_or_default(),
        };
        buckets.insert(k as u32, Point::new(lo.x - reach, lo.y - reach), Point::new(hi.x + reach, hi.y + reach));
    }
    let mut relief = Buckets::new(Point::new(x0, y0), Point::new(x0 + nx as i32 * h, y0 + ny as i32 * h), 1000);
    for (k, (_, shape)) in thermal_pads.iter().enumerate() {
        let (lo, hi) = shape.bounds();
        relief.insert(k as u32, Point::new(lo.x - reach, lo.y - reach), Point::new(hi.x + reach, hi.y + reach));
    }

    // The base field: how far inside the fill region each sample is, before the
    // thermal gaps, and the thermal gaps.
    let edge_clearance = f64::from(rules.edge_clearance);
    let keep = |p: Point| -> f32 {
        let mut v = polygon_depth(&zone.outline, p).min(polygon_depth(&board.outline, p) - edge_clearance) as f32;
        for &k in buckets.near(p) {
            let d = match &obstacles[k as usize] {
                Obstacle::Keep(shape, c) => shape.distance_to_point(p) - c,
                Obstacle::Zone(outline, c) => -polygon_depth(outline, p) - c,
            };
            v = v.min(d as f32);
        }
        v.min(cap)
    };
    let mut base = Grid { x0, y0, h, nx, ny, v: vec![f32::NEG_INFINITY; nx * ny] };
    let mut clear = vec![f32::NEG_INFINITY; nx * ny];
    for j in 1..ny - 1 {
        for i in 1..nx - 1 {
            let p = base.at(i, j);
            let k = keep(p);
            let mut v = k;
            for &t in relief.near(p) {
                v = v.min((thermal_pads[t as usize].1.distance_to_point(p) - gap) as f32);
            }
            clear[j * nx + i] = k - margin;
            base.v[j * nx + i] = v - margin;
        }
    }

    // Open by the minimum width: a sample stays where some sample at least half the
    // width deep is within half the width of it, measured to the edge of that deep
    // sample's own disc (the field is a distance, so a sample `d` deep has a disc of
    // radius `d - w/2` that is all deep).
    let window = ((half_width + 2. * f64::from(h)) / f64::from(h)).ceil() as isize;
    let mut opened = base.v.clone();
    let deep = |v: f32| f64::from(v) >= half_width;
    for j in 1..ny - 1 {
        for i in 1..nx - 1 {
            let v = base.v[j * nx + i];
            if v < 0. || deep(v) {
                continue;
            }
            let mut best = f64::INFINITY;
            for dj in -window..=window {
                for di in -window..=window {
                    let (a, b) = (i as isize + di, j as isize + dj);
                    if a < 0 || b < 0 || a >= nx as isize || b >= ny as isize {
                        continue;
                    }
                    let q = base.v[b as usize * nx + a as usize];
                    if !deep(q) {
                        continue;
                    }
                    let dist = f64::from(h) * ((di * di + dj * dj) as f64).sqrt();
                    best = best.min(dist - (f64::from(q) - half_width));
                }
            }
            // Slack of a micrometre so a wide region's own edge is not nibbled.
            let reach = (half_width - best + 1.) as f32;
            opened[j * nx + i] = v.min(reach);
        }
    }
    let mut field = Grid { v: opened, ..base };

    // Spokes: from each relieved pad's centre out along ±x and ±y, through the gap
    // and a spoke's width into the fill, laid only where the fill is there to meet.
    for (owner, shape) in &thermal_pads {
        let (lo, hi) = shape.bounds();
        let centre = shape.center();
        let half = [(hi.x - lo.x) / 2, (hi.y - lo.y) / 2];
        let mut spokes = 0u8;
        for (dx, dy) in [(1, 0), (-1, 0), (0, 1), (0, -1)] {
            let extent = f64::from(if dx != 0 { half[0] } else { half[1] });
            let meet = extent + gap + spoke_half.max(f64::from(h));
            let probe = Point::new(centre.x + (dx as f64 * meet) as i32, centre.y + (dy as f64 * meet) as i32);
            if field.sample(probe) <= 0. {
                continue;
            }
            let length = extent + gap + 2. * spoke_half;
            let end = Point::new(centre.x + (dx as f64 * length) as i32, centre.y + (dy as f64 * length) as i32);
            let spoke = Shape::segment(centre, end, zone.spoke_width);
            let (s0, s1) = spoke.bounds();
            let i0 = ((s0.x - x0) / h - 1).max(1) as usize;
            let j0 = ((s0.y - y0) / h - 1).max(1) as usize;
            let i1 = (((s1.x - x0) / h + 2) as usize).min(nx - 2);
            let j1 = (((s1.y - y0) / h + 2) as usize).min(ny - 2);
            for j in j0..=j1 {
                for i in i0..=i1 {
                    let p = field.at(i, j);
                    let s = (spoke_half - point_segment_distance(p, centre, end)) as f32;
                    let v = s.min(clear[j * nx + i] + margin) - margin;
                    let cell = &mut field.v[j * nx + i];
                    *cell = cell.max(v);
                }
            }
            spokes += 1;
        }
        report.thermals.push(Thermal { owner: *owner, at: centre, spokes });
    }

    // Trace, sort into pieces, drop the islands.
    let rings = contour(&field);
    let mut outers: Vec<Vec<Point>> = vec![];
    let mut holes: Vec<Vec<Point>> = vec![];
    for ring in rings {
        let ring = simplify_ring(&ring, SIMPLIFY);
        if ring.len() < 3 {
            continue;
        }
        let ring: Vec<Point> = ring
            .iter()
            .map(|&(x, y)| Point::new(x.round() as i32, y.round() as i32))
            .collect();
        let ring = super::simplify_path(ring);
        let area = ring_area(&ring);
        if area > 0. {
            outers.push(ring);
        } else if area < 0. {
            holes.push(ring);
        }
    }
    let mut pieces: Vec<FillPiece> = outers
        .into_iter()
        .map(|outer| FillPiece { outer, holes: vec![] })
        .collect();
    // Each hole goes to the smallest outer ring that holds it.
    for hole in holes {
        let probe = hole[0];
        let owner = pieces
            .iter()
            .enumerate()
            .filter(|(_, p)| polygon_contains(&p.outer, probe))
            .min_by(|a, b| ring_area(&a.1.outer).total_cmp(&ring_area(&b.1.outer)))
            .map(|(k, _)| k);
        if let Some(k) = owner {
            pieces[k].holes.push(hole);
        }
    }
    let before = pieces.len();
    pieces.retain(|piece| {
        let (pmin, pmax) = piece.bounds();
        same_net.iter().any(|&i| {
            let (lo, hi) = conn.items[i].shape.bounds();
            lo.x <= pmax.x && hi.x >= pmin.x && lo.y <= pmax.y && hi.y >= pmin.y
                && piece.touches(&conn.items[i].shape)
        })
    });
    report.islands_removed = before - pieces.len();
    // Largest first: the order a painter draws them in, and the one they are read in.
    pieces.sort_by(|a, b| b.area().total_cmp(&a.area()));
    report.pieces = pieces.len();
    report.area = pieces.iter().map(FillPiece::area).sum::<f64>() + 0.;
    if pieces.is_empty() {
        report.empty_because = Some(if before > 0 {
            format!(
                "no piece of it reaches any {0} copper (a pad, track or via of {0}), and a piece that joins nothing is an island, which is removed",
                zone.net
            )
        } else {
            "nothing is left of it once the clearances to other copper and to the board edge are kept".into()
        });
    }
    (pieces, report)
}

/// The zero contour of a field, as closed rings in board micrometres, the fill
/// (the positive side) on the left of each: outer rings turn positively, holes
/// negatively. Marching squares, with each crossing interpolated along its grid
/// edge and a saddle resolved by the cell's mean.
fn contour(grid: &Grid) -> Vec<Vec<(f64, f64)>> {
    let (nx, ny) = (grid.nx, grid.ny);
    let value = |i: usize, j: usize| {
        let v = grid.get(i, j);
        if v == 0. { -1e-6 } else { v }
    };
    // Edge ids: 2·(j·nx + i) is the edge from sample (i, j) to (i+1, j); one more is
    // the edge from (i, j) to (i, j+1).
    let horizontal = |i: usize, j: usize| 2 * (j * nx + i);
    let vertical = |i: usize, j: usize| 2 * (j * nx + i) + 1;
    let point = |edge: usize| -> (f64, f64) {
        let cell = edge / 2;
        let (i, j) = (cell % nx, cell / nx);
        let (i2, j2) = if edge % 2 == 0 { (i + 1, j) } else { (i, j + 1) };
        let (va, vb) = (f64::from(value(i, j)), f64::from(value(i2, j2)));
        let t = va / (va - vb);
        let (a, b) = (grid.at(i, j), grid.at(i2, j2));
        (
            f64::from(a.x) + t * f64::from(b.x - a.x),
            f64::from(a.y) + t * f64::from(b.y - a.y),
        )
    };
    let mut next: HashMap<usize, usize> = HashMap::new();
    for j in 0..ny - 1 {
        for i in 0..nx - 1 {
            let c = [value(i, j), value(i + 1, j), value(i + 1, j + 1), value(i, j + 1)];
            let inside = c.map(|v| v > 0.);
            let case = inside.iter().enumerate().fold(0, |acc, (k, &b)| acc | (usize::from(b) << k));
            if case == 0 || case == 15 {
                continue;
            }
            let edges = [horizontal(i, j), vertical(i + 1, j), horizontal(i, j + 1), vertical(i, j)];
            // Edge k runs from corner k to corner k+1: an exit where the fill ends
            // along it, an entry where it starts.
            let exits: Vec<usize> = (0..4).filter(|&k| inside[k] && !inside[(k + 1) % 4]).collect();
            if exits.len() == 1 {
                let entry = (0..4).find(|&k| !inside[k] && inside[(k + 1) % 4]).unwrap();
                next.insert(edges[exits[0]], edges[entry]);
            } else {
                let joined = c.iter().map(|&v| f64::from(v)).sum::<f64>() > 0.;
                for &k in &exits {
                    let entry = if joined { (k + 1) % 4 } else { (k + 3) % 4 };
                    next.insert(edges[k], edges[entry]);
                }
            }
        }
    }
    let mut starts: Vec<usize> = next.keys().copied().collect();
    starts.sort_unstable();
    let mut seen = std::collections::HashSet::new();
    let mut rings = vec![];
    for start in starts {
        if seen.contains(&start) {
            continue;
        }
        let mut ring = vec![];
        let mut edge = start;
        loop {
            if !seen.insert(edge) {
                break;
            }
            ring.push(point(edge));
            match next.get(&edge) {
                Some(&n) => edge = n,
                None => break,
            }
        }
        if ring.len() >= 3 {
            rings.push(ring);
        }
    }
    rings
}

/// A closed ring simplified to within `tolerance`: Douglas–Peucker on the two
/// halves between the first point and the point farthest from it.
fn simplify_ring(ring: &[(f64, f64)], tolerance: f64) -> Vec<(f64, f64)> {
    if ring.len() < 4 {
        return ring.to_vec();
    }
    let d2 = |a: (f64, f64), b: (f64, f64)| (a.0 - b.0).powi(2) + (a.1 - b.1).powi(2);
    let far = (1..ring.len())
        .max_by(|&a, &b| d2(ring[0], ring[a]).total_cmp(&d2(ring[0], ring[b])))
        .unwrap();
    let mut keep = vec![false; ring.len()];
    keep[0] = true;
    keep[far] = true;
    let closed: Vec<(f64, f64)> = ring.iter().copied().chain(std::iter::once(ring[0])).collect();
    let mut stack = vec![(0, far), (far, ring.len())];
    while let Some((a, b)) = stack.pop() {
        if b <= a + 1 {
            continue;
        }
        let (pa, pb) = (closed[a], closed[b]);
        let (dx, dy) = (pb.0 - pa.0, pb.1 - pa.1);
        let len = dx.hypot(dy);
        let mut worst = (0., a);
        for (k, &p) in closed.iter().enumerate().take(b).skip(a + 1) {
            let d = if len < 1e-9 {
                (p.0 - pa.0).hypot(p.1 - pa.1)
            } else {
                ((p.0 - pa.0) * dy - (p.1 - pa.1) * dx).abs() / len
            };
            if d > worst.0 {
                worst = (d, k);
            }
        }
        if worst.0 > tolerance {
            keep[worst.1] = true;
            stack.push((a, worst.1));
            stack.push((worst.1, b));
        }
    }
    ring.iter().zip(keep).filter(|(_, k)| *k).map(|(p, _)| *p).collect()
}

/// Trapezoids covering a set of rings by the even-odd rule, as triangles: each
/// strip between two consecutive vertex heights is cut where the rings' edges
/// cross it and filled between alternate crossings. Robust for any rings that do
/// not cross one another, holes included, which is what a fill is — and what
/// neither a fan nor ear clipping without hole bridging can draw.
///
/// The triangles are WATERTIGHT: where a strip meets the next, each side's edge
/// is split at every point the other side has on that line, and every point is
/// computed once, so two neighbouring triangles share their corners exactly and
/// no edge inside the fill is used by one triangle alone (the 3D view outlines a
/// hovered face by exactly those edges). A trapezoid with points on its top or
/// bottom edge is fanned from its centre.
pub fn fill_triangles(rings: &[&[Point]]) -> Vec<[(f64, f64); 3]> {
    type Edge = ((f64, f64), (f64, f64));
    let mut edges: Vec<Edge> = vec![];
    let mut ys: Vec<i32> = vec![];
    for ring in rings {
        for i in 0..ring.len() {
            let (a, b) = (ring[i], ring[(i + 1) % ring.len()]);
            ys.push(a.y);
            if a.y != b.y {
                let (a, b) = if a.y < b.y { (a, b) } else { (b, a) };
                edges.push(((f64::from(a.x), f64::from(a.y)), (f64::from(b.x), f64::from(b.y))));
            }
        }
    }
    ys.sort_unstable();
    ys.dedup();
    edges.sort_by(|a, b| a.0.1.total_cmp(&b.0.1));
    // An edge's x at `y`, EXACT at both of its ends, so the edge that ends at a
    // vertex and the one that starts there agree on it to the bit.
    let x_at = |e: &Edge, y: f64| {
        if y == e.0.1 {
            e.0.0
        } else if y == e.1.1 {
            e.1.0
        } else {
            e.0.0 + (e.1.0 - e.0.0) * (y - e.0.1) / (e.1.1 - e.0.1)
        }
    };
    // Each strip's filled spans: (x left, x right) at its top and at its bottom.
    let mut strips: Vec<Vec<[f64; 4]>> = Vec::with_capacity(ys.len());
    let mut active: Vec<usize> = vec![];
    let mut next_edge = 0;
    for w in ys.windows(2) {
        let (top, bottom) = (f64::from(w[0]), f64::from(w[1]));
        while next_edge < edges.len() && edges[next_edge].0.1 <= top {
            active.push(next_edge);
            next_edge += 1;
        }
        active.retain(|&e| edges[e].1.1 > top);
        let mid = (top + bottom) / 2.;
        let mut crossing: Vec<(f64, usize)> = active
            .iter()
            .filter(|&&e| edges[e].0.1 <= top && edges[e].1.1 >= bottom)
            .map(|&e| (x_at(&edges[e], mid), e))
            .collect();
        crossing.sort_by(|a, b| a.0.total_cmp(&b.0));
        strips.push(
            crossing
                .chunks_exact(2)
                .map(|pair| {
                    let (l, r) = (&edges[pair[0].1], &edges[pair[1].1]);
                    [x_at(l, top), x_at(r, top), x_at(l, bottom), x_at(r, bottom)]
                })
                .collect(),
        );
    }
    // Every point on each horizontal line, from the strips on both sides of it.
    let mut lines: Vec<Vec<f64>> = vec![vec![]; ys.len()];
    for (k, spans) in strips.iter().enumerate() {
        for span in spans {
            lines[k].extend([span[0], span[1]]);
            lines[k + 1].extend([span[2], span[3]]);
        }
    }
    for line in &mut lines {
        line.sort_by(f64::total_cmp);
        line.dedup();
    }
    let mut out = vec![];
    for (k, spans) in strips.iter().enumerate() {
        let (top, bottom) = (f64::from(ys[k]), f64::from(ys[k + 1]));
        for &[lt, rt, lb, rb] in spans {
            let mut polygon: Vec<(f64, f64)> = vec![(lt, top)];
            polygon.extend(lines[k].iter().filter(|&&x| x > lt && x < rt).map(|&x| (x, top)));
            polygon.push((rt, top));
            polygon.push((rb, bottom));
            polygon.extend(lines[k + 1].iter().rev().filter(|&&x| x > lb && x < rb).map(|&x| (x, bottom)));
            polygon.push((lb, bottom));
            polygon.dedup();
            if polygon.len() > 1 && polygon.first() == polygon.last() {
                polygon.pop();
            }
            let area2 = |a: (f64, f64), b: (f64, f64), c: (f64, f64)| {
                ((b.0 - a.0) * (c.1 - a.1) - (b.1 - a.1) * (c.0 - a.0)).abs()
            };
            if polygon.len() == 3 {
                if area2(polygon[0], polygon[1], polygon[2]) > 0. {
                    out.push([polygon[0], polygon[1], polygon[2]]);
                }
            } else if polygon.len() == 4 {
                for tri in [[polygon[0], polygon[1], polygon[2]], [polygon[0], polygon[2], polygon[3]]] {
                    if area2(tri[0], tri[1], tri[2]) > 0. {
                        out.push(tri);
                    }
                }
            } else if polygon.len() > 4 {
                let n = polygon.len() as f64;
                let centre = (
                    polygon.iter().map(|p| p.0).sum::<f64>() / n,
                    polygon.iter().map(|p| p.1).sum::<f64>() / n,
                );
                for i in 0..polygon.len() {
                    let tri = [centre, polygon[i], polygon[(i + 1) % polygon.len()]];
                    if area2(tri[0], tri[1], tri[2]) > 0. {
                        out.push(tri);
                    }
                }
            }
        }
    }
    out
}

/// A piece's rings as ONE ring, each hole joined to the outside by a cut-in: a
/// zero-width slit from the hole's leftmost vertex straight left to the nearest
/// edge, walked in and back out. The way a Gerber region carries a hole without a
/// clear-polarity step (which would also clear any copper drawn before it).
/// The outer ring comes out turning positively and each hole negatively.
pub fn fractured(piece: &FillPiece) -> Vec<Point> {
    let mut outer = piece.outer.clone();
    if ring_area(&outer) < 0. {
        outer.reverse();
    }
    let mut holes: Vec<Vec<Point>> = piece
        .holes
        .iter()
        .map(|h| {
            let mut h = h.clone();
            if ring_area(&h) > 0. {
                h.reverse();
            }
            // Start each hole at its leftmost vertex.
            let k = (0..h.len()).min_by_key(|&k| (h[k].x, h[k].y)).unwrap_or(0);
            h.rotate_left(k);
            h
        })
        .collect();
    holes.sort_by_key(|h| (h[0].x, h[0].y));
    for hole in holes {
        let v = hole[0];
        // The nearest edge of the ring so far that the ray from `v` leftwards meets.
        let mut best: Option<(f64, usize, Point)> = None;
        let n = outer.len();
        for i in 0..n {
            let (a, b) = (outer[i], outer[(i + 1) % n]);
            // Half-open, so a ring vertex level with `v` is met once.
            if (a.y <= v.y) == (b.y <= v.y) {
                continue;
            }
            let t = f64::from(v.y - a.y) / f64::from(b.y - a.y);
            let x = f64::from(a.x) + t * f64::from(b.x - a.x);
            if x > f64::from(v.x) {
                continue;
            }
            let d = f64::from(v.x) - x;
            if best.is_none_or(|(bd, ..)| d < bd) {
                best = Some((d, i, Point::new(x.round() as i32, v.y)));
            }
        }
        let Some((_, i, at)) = best else { continue };
        let mut spliced = Vec::with_capacity(n + hole.len() + 3);
        spliced.extend_from_slice(&outer[..=i]);
        if outer[i] != at {
            spliced.push(at);
        }
        spliced.extend(hole.iter().copied());
        spliced.push(v);
        spliced.push(at);
        spliced.extend_from_slice(&outer[i + 1..]);
        outer = spliced;
    }
    outer
}

/// A zone's settings as a line: "GND on B.Cu".
pub fn describe(zone: &Zone, layer_count: u8) -> String {
    format!("{} zone on {}", zone.net, super::layer_name(zone.layer, layer_count))
}
