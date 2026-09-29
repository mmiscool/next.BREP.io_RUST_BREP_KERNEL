//! Printed circuit board layout: footprints, copper, connectivity, and design checks.
//!
//! Board coordinates use the schematic's integer micrometres with positive Y down.
//! Copper connectivity is derived purely from geometry and net identity comes from
//! the schematic netlist, so the schematic remains the single source of truth.
use crate::{Document, Netlist, Point, Uuid, footprint};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};

/// Courtyard clearance added around footprint bounds.
pub const COURTYARD_MARGIN: i32 = 250;
/// Gap between parts and from the board edge during automatic placement.
const PLACEMENT_SPACING: i32 = 2000;
/// Gap a part added to a board already laid out keeps from other courtyards and
/// from the outline ([`Document::seat_in_free_room`]).
const SEAT_GAP: i32 = 1000;

pub mod net_classes;
pub mod zones;
pub use net_classes::{ClassReason, DEFAULT_CLASS, NetClass};
pub use zones::{FillPiece, Zone};

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum PadShape {
    #[default]
    Rect,
    Circle,
    Oval,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Pad {
    /// Matches a symbol pin number. Empty for mechanical pads.
    pub number: String,
    pub at: Point,
    /// Width (x) and height (y) in footprint coordinates.
    pub size: Point,
    #[serde(default)]
    pub shape: PadShape,
    /// Through-hole drill diameter; `None` for surface-mount pads.
    #[serde(default)]
    pub drill: Option<i32>,
    /// Whether a drilled hole is plated. Unplated holes go to the NPTH drill file.
    #[serde(default = "plated_by_default")]
    pub plated: bool,
}
fn plated_by_default() -> bool {
    true
}

/// The 3D model a footprint names, as KiCad's `(model …)` entry gives it: the file and
/// its placement on the pads. The numbers are carried through as the file authored them,
/// millimetres for the offset and degrees for the rotation, for a host to interpret.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Model {
    /// The file the footprint names, often with a variable such as
    /// `${KICAD9_3DMODEL_DIR}` that the host resolves.
    pub path: String,
    #[serde(default)]
    pub offset: [f64; 3],
    #[serde(default = "unit_scale")]
    pub scale: [f64; 3],
    #[serde(default)]
    pub rotation: [f64; 3],
}
fn unit_scale() -> [f64; 3] {
    [1., 1., 1.]
}
impl Model {
    /// The same model as a STEP file. KiCad's libraries name a `.wrl` mesh for rendering
    /// and keep a `.step` of the same name beside it, which carries the solid geometry;
    /// any other path is returned unchanged.
    pub fn step_path(&self) -> String {
        match self.path.strip_suffix(".wrl") {
            Some(stem) => format!("{stem}.step"),
            None => self.path.clone(),
        }
    }
}
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Footprint {
    pub name: String,
    pub pads: Vec<Pad>,
    /// Silkscreen polylines in footprint coordinates.
    #[serde(default)]
    pub silk: Vec<Vec<Point>>,
    /// The 3D model this footprint names, if it has one.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub model: Option<Model>,
}
impl Footprint {
    /// Check a footprint on its own, with messages that name the offending pad: the
    /// AUTHORING rule, [`Footprint::validate_placeable`] plus a name.
    pub fn validate(&self) -> Result<(), String> {
        if self.name.trim().is_empty() {
            return Err("Enter a footprint name.".into());
        }
        self.validate_placeable()
    }
    /// Whether a footprint can sit on a board: every check of [`Footprint::validate`]
    /// except the name. A name is a label, and a copy of a part's pads that has none
    /// is still copper a board can carry; refusing it there refused the whole document,
    /// so one unnamed part in an assembly stopped every edit to the sheet and the board
    /// from being stored (the eCAD workflow audit, issue 3).
    pub fn validate_placeable(&self) -> Result<(), String> {
        for pad in &self.pads {
            let name = if pad.number.is_empty() {
                "An unnumbered pad".to_owned()
            } else {
                format!("Pad {}", pad.number)
            };
            if pad.size.x <= 0 || pad.size.y <= 0 {
                return Err(format!("{name} needs a positive width and height."));
            }
            if pad
                .drill
                .is_some_and(|d| d <= 0 || d > pad.size.x.max(pad.size.y))
            {
                return Err(format!("{name} has a drill larger than the pad."));
            }
        }
        if self.silk.iter().any(|line| line.len() < 2) {
            return Err("Silkscreen lines need at least two points.".into());
        }
        if let Some(model) = &self.model {
            let numbers = model
                .offset
                .iter()
                .chain(&model.scale)
                .chain(&model.rotation);
            if model.path.trim().is_empty() || numbers.into_iter().any(|v| !v.is_finite()) {
                return Err("The 3D model needs a file and finite placement numbers.".into());
            }
        }
        let id = Uuid::nil();
        let board = Board {
            placements: vec![Placement {
                component: id,
                footprint: self.clone(),
                at: Point::new(0, 0),
                rotation: 0,
                bottom: false,
            }],
            ..Board::default()
        };
        board.validate(&BTreeSet::from([id]))
    }
    /// Bounds of pads and silkscreen in footprint coordinates.
    pub fn bounds(&self) -> (Point, Point) {
        let mut points: Vec<Point> = self
            .pads
            .iter()
            .flat_map(|p| {
                let (min, max) = Shape::pad(p.at, p.size, PadShape::Rect).bounds();
                [min, max]
            })
            .collect();
        points.extend(self.silk.iter().flatten().copied());
        bounds_of(&points).unwrap_or_default()
    }
}

/// One footprint instance, bound to a schematic component by ID.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Placement {
    pub component: Uuid,
    pub footprint: Footprint,
    pub at: Point,
    #[serde(default)]
    pub rotation: u8,
    /// Mounted on the bottom side: mirrored in X, and SMD pads use the last layer.
    #[serde(default)]
    pub bottom: bool,
}
impl Placement {
    pub fn transform(&self, p: Point) -> Point {
        let p = if self.bottom {
            Point::new(-p.x, p.y)
        } else {
            p
        };
        p.rotate(self.rotation).offset(self.at)
    }
    pub fn pad_shape(&self, pad: &Pad) -> Shape {
        let size = if self.rotation % 2 == 1 {
            Point::new(pad.size.y, pad.size.x)
        } else {
            pad.size
        };
        Shape::pad(self.transform(pad.at), size, pad.shape)
    }
    /// Inclusive copper layer range of a pad.
    pub fn pad_layers(&self, pad: &Pad, layer_count: u8) -> (u8, u8) {
        let last = layer_count.saturating_sub(1);
        match (pad.drill, self.bottom) {
            (Some(_), _) => (0, last),
            (None, false) => (0, 0),
            (None, true) => (last, last),
        }
    }
    /// World-space courtyard rectangle.
    pub fn courtyard(&self) -> (Point, Point) {
        let (min, max) = self.footprint.bounds();
        let corners = [min, max, Point::new(min.x, max.y), Point::new(max.x, min.y)]
            .map(|p| self.transform(p));
        let (min, max) = bounds_of(&corners).unwrap_or_default();
        (
            Point::new(min.x - COURTYARD_MARGIN, min.y - COURTYARD_MARGIN),
            Point::new(max.x + COURTYARD_MARGIN, max.y + COURTYARD_MARGIN),
        )
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Track {
    pub id: Uuid,
    pub layer: u8,
    pub width: i32,
    /// The width was set on this track itself, so the net's width rule leaves it alone.
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub fixed_width: bool,
    pub points: Vec<Point>,
}
impl Track {
    /// Build a track, dropping repeated and collinear interior points.
    pub fn new(layer: u8, width: i32, points: Vec<Point>) -> Self {
        Self {
            id: Uuid::new_v4(),
            layer,
            width,
            fixed_width: false,
            points: simplify_path(points),
        }
    }
    pub fn segments(&self) -> impl Iterator<Item = Shape> + '_ {
        self.points
            .windows(2)
            .map(|s| Shape::segment(s[0], s[1], self.width))
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Via {
    pub id: Uuid,
    pub at: Point,
    pub diameter: i32,
    pub drill: i32,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct DesignRules {
    /// The Default net class's track width, clearance and via ([`net_classes`]).
    pub track_width: i32,
    pub clearance: i32,
    pub via_diameter: i32,
    pub via_drill: i32,
    /// Minimum distance from copper to the board outline.
    pub edge_clearance: i32,
    /// Autorouter grid pitch.
    pub routing_grid: i32,
    /// Track width overrides by exact net name, e.g. wider supply nets.
    #[serde(default)]
    pub net_widths: BTreeMap<String, i32>,
    /// Solder mask openings extend this far beyond each pad.
    #[serde(default = "default_mask_expansion")]
    pub mask_expansion: i32,
    /// How the autorouter uses vias. Boards saved before this existed get the default.
    #[serde(default)]
    pub autoroute_vias: ViaPolicy,
    /// The net classes besides Default, in pattern order ([`net_classes`]). Written
    /// only when there are some, so a board with none saves as it did before.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub net_classes: Vec<NetClass>,
    /// Explicit net → class assignments, which beat every pattern.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub net_class_of: BTreeMap<String, String>,
}
fn default_mask_expansion() -> i32 {
    50
}

/// How the autorouter trades vias against track length.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ViaPolicy {
    /// A via is an ordinary cost, taken whenever it shortens the route enough.
    Allow,
    /// Route as `Allow` does, then reroute every net that has vias against the
    /// finished board, looking first for a route with none, and keep whichever
    /// copper has fewer. Connects the same nets as `Allow`, more slowly.
    #[default]
    Avoid,
    /// Never place a via. Connections that cannot be made on one layer stay unrouted.
    Never,
}
impl Default for DesignRules {
    fn default() -> Self {
        Self {
            track_width: 250,
            clearance: 200,
            via_diameter: 600,
            via_drill: 300,
            edge_clearance: 300,
            routing_grid: 100,
            net_widths: BTreeMap::new(),
            mask_expansion: default_mask_expansion(),
            autoroute_vias: ViaPolicy::default(),
            net_classes: vec![],
            net_class_of: BTreeMap::new(),
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Board {
    /// Closed board outline polygon.
    pub outline: Vec<Point>,
    /// Copper layers; layer 0 is the top (F.Cu), the last is the bottom (B.Cu).
    pub layer_count: u8,
    #[serde(default)]
    pub placements: Vec<Placement>,
    #[serde(default)]
    pub tracks: Vec<Track>,
    #[serde(default)]
    pub vias: Vec<Via>,
    #[serde(default)]
    pub rules: DesignRules,
    /// Copper zones, in drawing order, which is also their fill priority. Written
    /// only when there are some, so a board with none serializes exactly as it did
    /// before zones existed (the host compares the block byte for byte).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub zones: Vec<Zone>,
}
impl Default for Board {
    fn default() -> Self {
        Self {
            outline: rectangle(Point::new(0, 0), Point::new(60_000, 40_000)),
            layer_count: 2,
            placements: vec![],
            tracks: vec![],
            vias: vec![],
            rules: DesignRules::default(),
            zones: vec![],
        }
    }
}

/// The substrate's thickness in board micrometres — 1.6 mm, the board every
/// fabricator quotes by default.
///
/// A CONSTANT rather than a [`Board`] field on purpose. A field would serialize
/// into the `pcb` block, and the eCAD host compares the block it holds against
/// the stored one byte for byte to decide whether the editor is in step — a new
/// always-serialized field puts every block written before it out of step (the
/// `Pin::unit` lesson, 2026-09-20). Until a board needs its own stack-up, one
/// number the app and the 3D build share is honest and costs nothing.
pub const SUBSTRATE_THICKNESS: i32 = 1_600;

/// Copper foil thickness in board micrometres — 35 µm, "1 oz" copper. Also the
/// wall thickness of a plated via barrel.
pub const COPPER_THICKNESS: i32 = 35;

/// The z of copper layer `layer`'s own plane, in board micrometres, with the
/// TOP copper plane at 0 and +z out of the board's top face — the frame
/// [`Placement`] poses are already written in.
///
/// Layer 0 (`F.Cu`) is 0 and the last (`B.Cu`) is `-SUBSTRATE_THICKNESS`; the
/// inner layers are spread evenly between them, so the LAYER COUNT decides the
/// stack exactly as it decides the names ([`layer_name`]). A one-layer board
/// has only layer 0, at 0.
pub fn layer_z(layer: u8, layer_count: u8) -> i32 {
    let last = layer_count.saturating_sub(1);
    if layer == 0 || last == 0 {
        return 0;
    }
    let layer = layer.min(last);
    -(SUBSTRATE_THICKNESS as i64 * i64::from(layer) / i64::from(last)) as i32
}

/// KiCad-style copper layer name.
pub fn layer_name(layer: u8, layer_count: u8) -> String {
    if layer == 0 {
        "F.Cu".into()
    } else if layer + 1 >= layer_count {
        "B.Cu".into()
    } else {
        format!("In{layer}.Cu")
    }
}

pub fn rectangle(min: Point, max: Point) -> Vec<Point> {
    vec![min, Point::new(max.x, min.y), max, Point::new(min.x, max.y)]
}

/// A copper primitive for clearance and connectivity calculations.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Shape {
    Rect {
        min: Point,
        max: Point,
    },
    /// A segment swept by a disc. A circle has `a == b`.
    Capsule {
        a: Point,
        b: Point,
        radius: i32,
    },
}
impl Shape {
    pub fn pad(center: Point, size: Point, shape: PadShape) -> Self {
        let half = Point::new(size.x / 2, size.y / 2);
        match shape {
            PadShape::Rect => {
                let min = Point::new(center.x - half.x, center.y - half.y);
                Self::Rect {
                    min,
                    max: Point::new(min.x + size.x, min.y + size.y),
                }
            }
            PadShape::Circle => Self::circle(center, size.x.min(size.y) / 2),
            PadShape::Oval => {
                let radius = size.x.min(size.y) / 2;
                let reach = Point::new((half.x - radius).max(0), (half.y - radius).max(0));
                Self::Capsule {
                    a: Point::new(center.x - reach.x, center.y - reach.y),
                    b: Point::new(center.x + reach.x, center.y + reach.y),
                    radius,
                }
            }
        }
    }
    pub fn circle(center: Point, radius: i32) -> Self {
        Self::Capsule {
            a: center,
            b: center,
            radius,
        }
    }
    pub fn segment(a: Point, b: Point, width: i32) -> Self {
        Self::Capsule {
            a,
            b,
            radius: width / 2,
        }
    }
    pub fn bounds(&self) -> (Point, Point) {
        match *self {
            Self::Rect { min, max } => (min, max),
            Self::Capsule { a, b, radius } => (
                Point::new(a.x.min(b.x) - radius, a.y.min(b.y) - radius),
                Point::new(a.x.max(b.x) + radius, a.y.max(b.y) + radius),
            ),
        }
    }
    pub fn center(&self) -> Point {
        let (min, max) = self.bounds();
        Point::new(
            ((i64::from(min.x) + i64::from(max.x)) / 2) as i32,
            ((i64::from(min.y) + i64::from(max.y)) / 2) as i32,
        )
    }
    /// Distance from a point to the shape; zero inside.
    pub fn distance_to_point(&self, p: Point) -> f64 {
        match *self {
            Self::Rect { min, max } => point_rect_distance(p, min, max),
            Self::Capsule { a, b, radius } => {
                (point_segment_distance(p, a, b) - f64::from(radius)).max(0.)
            }
        }
    }
    pub fn contains(&self, p: Point) -> bool {
        self.distance_to_point(p) <= 0.
    }
    /// Gap between two shapes; zero when they touch or overlap.
    pub fn distance(&self, other: &Self) -> f64 {
        match (*self, *other) {
            (Self::Rect { min: a0, max: a1 }, Self::Rect { min: b0, max: b1 }) => {
                let dx = (i64::from(a0.x) - i64::from(b1.x))
                    .max(i64::from(b0.x) - i64::from(a1.x))
                    .max(0) as f64;
                let dy = (i64::from(a0.y) - i64::from(b1.y))
                    .max(i64::from(b0.y) - i64::from(a1.y))
                    .max(0) as f64;
                dx.hypot(dy)
            }
            (Self::Rect { min, max }, Self::Capsule { a, b, radius })
            | (Self::Capsule { a, b, radius }, Self::Rect { min, max }) => {
                (segment_rect_distance(a, b, min, max) - f64::from(radius)).max(0.)
            }
            (
                Self::Capsule {
                    a: a0,
                    b: a1,
                    radius: ra,
                },
                Self::Capsule {
                    a: b0,
                    b: b1,
                    radius: rb,
                },
            ) => (segment_distance(a0, a1, b0, b1) - f64::from(ra) - f64::from(rb)).max(0.),
        }
    }
}

fn cross(o: Point, a: Point, b: Point) -> i64 {
    (i64::from(a.x) - i64::from(o.x)) * (i64::from(b.y) - i64::from(o.y))
        - (i64::from(a.y) - i64::from(o.y)) * (i64::from(b.x) - i64::from(o.x))
}
fn within_box(a: Point, b: Point, p: Point) -> bool {
    p.x >= a.x.min(b.x) && p.x <= a.x.max(b.x) && p.y >= a.y.min(b.y) && p.y <= a.y.max(b.y)
}
pub fn segments_intersect(a: Point, b: Point, c: Point, d: Point) -> bool {
    let d1 = cross(c, d, a);
    let d2 = cross(c, d, b);
    let d3 = cross(a, b, c);
    let d4 = cross(a, b, d);
    if d1.signum() * d2.signum() < 0 && d3.signum() * d4.signum() < 0 {
        return true;
    }
    (d1 == 0 && within_box(c, d, a))
        || (d2 == 0 && within_box(c, d, b))
        || (d3 == 0 && within_box(a, b, c))
        || (d4 == 0 && within_box(a, b, d))
}
pub fn point_segment_distance(p: Point, a: Point, b: Point) -> f64 {
    let (px, py) = (f64::from(p.x), f64::from(p.y));
    let (ax, ay) = (f64::from(a.x), f64::from(a.y));
    let (dx, dy) = (f64::from(b.x) - ax, f64::from(b.y) - ay);
    let len2 = dx * dx + dy * dy;
    let t = if len2 == 0. {
        0.
    } else {
        (((px - ax) * dx + (py - ay) * dy) / len2).clamp(0., 1.)
    };
    (px - ax - t * dx).hypot(py - ay - t * dy)
}
fn segment_distance(a: Point, b: Point, c: Point, d: Point) -> f64 {
    if segments_intersect(a, b, c, d) {
        return 0.;
    }
    point_segment_distance(a, c, d)
        .min(point_segment_distance(b, c, d))
        .min(point_segment_distance(c, a, b))
        .min(point_segment_distance(d, a, b))
}
fn point_rect_distance(p: Point, min: Point, max: Point) -> f64 {
    let dx = (i64::from(min.x) - i64::from(p.x))
        .max(i64::from(p.x) - i64::from(max.x))
        .max(0) as f64;
    let dy = (i64::from(min.y) - i64::from(p.y))
        .max(i64::from(p.y) - i64::from(max.y))
        .max(0) as f64;
    dx.hypot(dy)
}
fn segment_rect_distance(a: Point, b: Point, min: Point, max: Point) -> f64 {
    let corners = rectangle(min, max);
    if within_box(min, max, a)
        || within_box(min, max, b)
        || (0..4).any(|i| segments_intersect(a, b, corners[i], corners[(i + 1) % 4]))
    {
        return 0.;
    }
    corners
        .iter()
        .map(|c| point_segment_distance(*c, a, b))
        .fold(
            point_rect_distance(a, min, max).min(point_rect_distance(b, min, max)),
            f64::min,
        )
}
/// Even-odd point-in-polygon test.
pub fn polygon_contains(polygon: &[Point], p: Point) -> bool {
    let (px, py) = (f64::from(p.x), f64::from(p.y));
    let mut inside = false;
    for (i, a) in polygon.iter().enumerate() {
        let b = polygon[(i + 1) % polygon.len()];
        let (ax, ay, bx, by) = (
            f64::from(a.x),
            f64::from(a.y),
            f64::from(b.x),
            f64::from(b.y),
        );
        if (ay > py) != (by > py) && px < ax + (py - ay) * (bx - ax) / (by - ay) {
            inside = !inside;
        }
    }
    inside
}
fn bounds_of(points: &[Point]) -> Option<(Point, Point)> {
    let first = *points.first()?;
    Some(points.iter().fold((first, first), |(min, max), p| {
        (
            Point::new(min.x.min(p.x), min.y.min(p.y)),
            Point::new(max.x.max(p.x), max.y.max(p.y)),
        )
    }))
}
/// Remove repeated points and interior points that continue in the same direction.
pub fn simplify_path(points: Vec<Point>) -> Vec<Point> {
    let mut clean: Vec<Point> = vec![];
    for p in points {
        if clean.last() == Some(&p) {
            continue;
        }
        if clean.len() >= 2 {
            let a = clean[clean.len() - 2];
            let b = clean[clean.len() - 1];
            let forward = (i64::from(b.x) - i64::from(a.x)) * (i64::from(p.x) - i64::from(b.x))
                + (i64::from(b.y) - i64::from(a.y)) * (i64::from(p.y) - i64::from(b.y));
            if cross(a, b, p) == 0 && forward > 0 {
                clean.pop();
            }
        }
        clean.push(p);
    }
    clean
}

/// What a copper primitive belongs to.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum CopperRef {
    Pad { placement: usize, pad: usize },
    Track { track: usize, segment: usize },
    Via(usize),
}
#[derive(Clone, Debug)]
pub struct CopperItem {
    pub owner: CopperRef,
    pub shape: Shape,
    /// Inclusive layer range.
    pub layers: (u8, u8),
}
impl CopperItem {
    fn shares_layer(&self, other: &Self) -> bool {
        self.layers.0 <= other.layers.1 && other.layers.0 <= self.layers.1
    }
}

/// Copper islands and the schematic nets each island carries.
#[derive(Clone, Debug, Default)]
pub struct Connectivity {
    pub items: Vec<CopperItem>,
    /// Island index for each item.
    pub islands: Vec<usize>,
    pub island_count: usize,
    /// Schematic net of each pad item.
    pub item_nets: Vec<Option<String>>,
    pub island_nets: Vec<BTreeSet<String>>,
    /// Island index of each piece of each zone's fill, by zone then piece: a fill
    /// is copper, and joins every item it touches.
    pub zone_islands: Vec<Vec<usize>>,
}
impl Connectivity {
    /// The single net carried by an island, if it carries exactly one.
    pub fn island_net(&self, island: usize) -> Option<&str> {
        let nets = &self.island_nets[island];
        (nets.len() == 1).then(|| nets.iter().next().unwrap().as_str())
    }
}

/// An unrouted connection between two copper islands of one net.
#[derive(Clone, Debug, PartialEq)]
pub struct Airwire {
    pub net: String,
    pub a: Point,
    pub b: Point,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum ViolationKind {
    Short,
    Clearance,
    /// A track narrower than its net's width: its own width rule, else its class's.
    TrackWidth,
    BoardEdge,
    MissingPad,
    Courtyard,
    /// A via with no net that zones of two nets claim, and which is stitched to
    /// neither ([`Board::contested_vias`]).
    Stitching,
    Unrouted,
}
#[derive(Clone, Debug, PartialEq)]
pub struct Violation {
    pub kind: ViolationKind,
    pub message: String,
    pub at: Point,
}

struct Dsu(Vec<usize>);
impl Dsu {
    fn new(n: usize) -> Self {
        Self((0..n).collect())
    }
    fn root(&mut self, mut i: usize) -> usize {
        while self.0[i] != i {
            self.0[i] = self.0[self.0[i]];
            i = self.0[i];
        }
        i
    }
    fn join(&mut self, a: usize, b: usize) -> bool {
        let (a, b) = (self.root(a), self.root(b));
        if a != b {
            self.0[a.max(b)] = a.min(b);
        }
        a != b
    }
}

/// Visit item pairs whose bounds, expanded by `slack`, overlap on a shared layer.
fn near_pairs(items: &[CopperItem], slack: i32, mut visit: impl FnMut(usize, usize)) {
    let bounds: Vec<_> = items.iter().map(|i| i.shape.bounds()).collect();
    let mut order: Vec<usize> = (0..items.len()).collect();
    order.sort_by_key(|&i| (bounds[i].0.x, i));
    for (k, &i) in order.iter().enumerate() {
        let (min, max) = bounds[i];
        for &j in &order[k + 1..] {
            let (other_min, other_max) = bounds[j];
            if other_min.x > max.x.saturating_add(slack) {
                break;
            }
            if other_min.y > max.y.saturating_add(slack)
                || min.y > other_max.y.saturating_add(slack)
                || !items[i].shares_layer(&items[j])
            {
                continue;
            }
            visit(i.min(j), i.max(j));
        }
    }
}

/// Schematic net name for each (component, pin number).
pub fn pin_nets(netlist: &Netlist) -> BTreeMap<(Uuid, String), String> {
    netlist
        .nets
        .iter()
        .flat_map(|n| {
            n.pins
                .iter()
                .map(|p| ((p.component_id, p.number.clone()), n.name.clone()))
        })
        .collect()
}

impl Board {
    pub fn outline_bounds(&self) -> (Point, Point) {
        bounds_of(&self.outline).unwrap_or_default()
    }
    /// True when the outline is an axis-aligned rectangle.
    pub fn outline_is_rectangle(&self) -> bool {
        let (min, max) = self.outline_bounds();
        self.outline.len() == 4
            && self
                .outline
                .iter()
                .all(|p| (p.x == min.x || p.x == max.x) && (p.y == min.y || p.y == max.y))
            && min.x < max.x
            && min.y < max.y
    }
    /// Even-odd point-in-polygon test for the outline.
    pub fn outline_contains(&self, p: Point) -> bool {
        polygon_contains(&self.outline, p)
    }
    pub fn outline_edges(&self) -> impl Iterator<Item = (Point, Point)> + '_ {
        let n = self.outline.len();
        (0..n).map(move |i| (self.outline[i], self.outline[(i + 1) % n]))
    }
    /// Change the number of copper layers. Bottom-layer copper stays on the bottom;
    /// changes that would strand inner-layer tracks, or leave a single-layer board
    /// with vias or bottom-side copper, are refused.
    pub fn set_layer_count(&mut self, count: u8) -> Result<(), String> {
        if !(1..=32).contains(&count) {
            return Err("Boards need between 1 and 32 copper layers".into());
        }
        let old_last = self.layer_count.saturating_sub(1);
        let new_last = count - 1;
        if self
            .tracks
            .iter()
            .any(|t| t.layer != 0 && t.layer != old_last && t.layer >= new_last)
        {
            return Err("Move or delete the tracks on removed inner layers first".into());
        }
        if self
            .zones
            .iter()
            .any(|z| z.layer != 0 && z.layer != old_last && z.layer >= new_last)
        {
            return Err("Move or delete the zones on removed inner layers first".into());
        }
        if count == 1 && self.zones.iter().any(|z| z.layer != 0) {
            return Err("A single-layer board cannot keep bottom zones".into());
        }
        if count == 1
            && (!self.vias.is_empty()
                || self.placements.iter().any(|p| p.bottom)
                || self.tracks.iter().any(|t| t.layer != 0))
        {
            return Err(
                "A single-layer board cannot keep vias, bottom tracks, or bottom-side parts".into(),
            );
        }
        for track in &mut self.tracks {
            if track.layer == old_last && old_last != 0 {
                track.layer = new_last;
            }
        }
        for zone in &mut self.zones {
            if zone.layer == old_last && old_last != 0 {
                zone.layer = new_last;
            }
        }
        self.layer_count = count;
        Ok(())
    }
    pub fn placement(&self, component: Uuid) -> Option<&Placement> {
        self.placements.iter().find(|p| p.component == component)
    }
    /// Every copper primitive on the board.
    pub fn copper(&self) -> Vec<CopperItem> {
        let last = self.layer_count.saturating_sub(1);
        let mut items = vec![];
        for (pi, placement) in self.placements.iter().enumerate() {
            for (pad_index, pad) in placement.footprint.pads.iter().enumerate() {
                items.push(CopperItem {
                    owner: CopperRef::Pad {
                        placement: pi,
                        pad: pad_index,
                    },
                    shape: placement.pad_shape(pad),
                    layers: placement.pad_layers(pad, self.layer_count),
                });
            }
        }
        for (ti, track) in self.tracks.iter().enumerate() {
            for (segment, shape) in track.segments().enumerate() {
                items.push(CopperItem {
                    owner: CopperRef::Track { track: ti, segment },
                    shape,
                    layers: (track.layer, track.layer),
                });
            }
        }
        for (vi, via) in self.vias.iter().enumerate() {
            items.push(CopperItem {
                owner: CopperRef::Via(vi),
                shape: Shape::circle(via.at, via.diameter / 2),
                layers: (0, last),
            });
        }
        items
    }
    /// Copper islands and their nets, a zone's fill joining whatever it touches.
    pub fn connectivity(&self, netlist: &Netlist) -> Connectivity {
        self.connectivity_of(netlist, true)
    }
    /// [`Self::connectivity`], with or without the zones' fills: a fill is made
    /// against the copper WITHOUT any fill, so a stale one never decides the net
    /// of a track.
    pub fn connectivity_of(&self, netlist: &Netlist, with_zones: bool) -> Connectivity {
        let items = self.copper();
        let pins = pin_nets(netlist);
        let pieces: Vec<(usize, &FillPiece)> = if with_zones {
            self.zones
                .iter()
                .enumerate()
                .flat_map(|(z, zone)| zone.fill.iter().map(move |p| (z, p)))
                .collect()
        } else {
            vec![]
        };
        let mut dsu = Dsu::new(items.len() + pieces.len());
        near_pairs(&items, 0, |i, j| {
            if items[i].shape.distance(&items[j].shape) <= 0.5 {
                dsu.join(i, j);
            }
        });
        for (k, (z, piece)) in pieces.iter().enumerate() {
            let layer = self.zones[*z].layer;
            let (min, max) = piece.bounds();
            for (i, item) in items.iter().enumerate() {
                let (lo, hi) = item.shape.bounds();
                if item.layers.0 <= layer
                    && layer <= item.layers.1
                    && lo.x <= max.x
                    && hi.x >= min.x
                    && lo.y <= max.y
                    && hi.y >= min.y
                    && piece.touches(&item.shape)
                {
                    dsu.join(i, items.len() + k);
                }
            }
        }
        let mut numbering = BTreeMap::new();
        let mut all: Vec<usize> = (0..items.len() + pieces.len())
            .map(|i| {
                let next = numbering.len();
                *numbering.entry(dsu.root(i)).or_insert(next)
            })
            .collect();
        let piece_islands = all.split_off(items.len());
        let islands = all;
        let mut zone_islands: Vec<Vec<usize>> = if with_zones {
            self.zones.iter().map(|z| Vec::with_capacity(z.fill.len())).collect()
        } else {
            vec![]
        };
        for ((z, _), island) in pieces.iter().zip(piece_islands) {
            zone_islands[*z].push(island);
        }
        let item_nets: Vec<Option<String>> = items
            .iter()
            .map(|item| match item.owner {
                CopperRef::Pad { placement, pad } => {
                    let placement = &self.placements[placement];
                    let number = &placement.footprint.pads[pad].number;
                    pins.get(&(placement.component, number.clone())).cloned()
                }
                _ => None,
            })
            .collect();
        let mut island_nets = vec![BTreeSet::new(); numbering.len()];
        for (island, net) in islands.iter().zip(&item_nets) {
            if let Some(net) = net {
                island_nets[*island].insert(net.clone());
            }
        }
        Connectivity {
            island_count: numbering.len(),
            items,
            islands,
            item_nets,
            island_nets,
            zone_islands,
        }
    }
    /// Minimum spanning connections between the unjoined copper islands of each net.
    pub fn ratsnest(&self, conn: &Connectivity) -> Vec<Airwire> {
        let mut net_pads: BTreeMap<&str, Vec<usize>> = BTreeMap::new();
        for (i, net) in conn.item_nets.iter().enumerate() {
            if let Some(net) = net {
                net_pads.entry(net).or_default().push(i);
            }
        }
        let mut island_anchors: BTreeMap<usize, Vec<Point>> = BTreeMap::new();
        for (i, item) in conn.items.iter().enumerate() {
            let anchors = island_anchors.entry(conn.islands[i]).or_default();
            match item.owner {
                CopperRef::Track { .. } => {
                    if let Shape::Capsule { a, b, .. } = item.shape {
                        anchors.extend([a, b]);
                    }
                }
                CopperRef::Via(_) => anchors.push(item.shape.center()),
                CopperRef::Pad { .. } => {}
            }
        }
        let mut airwires = vec![];
        for (net, pads) in net_pads {
            let islands: BTreeSet<usize> = pads.iter().map(|&i| conn.islands[i]).collect();
            if islands.len() < 2 {
                continue;
            }
            let mut anchors: Vec<(usize, Point)> = pads
                .iter()
                .map(|&i| (conn.islands[i], conn.items[i].shape.center()))
                .collect();
            let extra: Vec<(usize, Point)> = islands
                .iter()
                .flat_map(|&island| {
                    island_anchors
                        .get(&island)
                        .into_iter()
                        .flatten()
                        .map(move |p| (island, *p))
                })
                .collect();
            if anchors.len() + extra.len() <= 400 {
                anchors.extend(extra);
            }
            anchors.sort();
            anchors.dedup();
            let mut pairs = vec![];
            for i in 0..anchors.len() {
                for j in i + 1..anchors.len() {
                    if anchors[i].0 != anchors[j].0 {
                        let dx = i64::from(anchors[i].1.x) - i64::from(anchors[j].1.x);
                        let dy = i64::from(anchors[i].1.y) - i64::from(anchors[j].1.y);
                        pairs.push((dx * dx + dy * dy, i, j));
                    }
                }
            }
            pairs.sort_unstable();
            let index: BTreeMap<usize, usize> =
                islands.iter().enumerate().map(|(i, &s)| (s, i)).collect();
            let mut dsu = Dsu::new(islands.len());
            let mut joined = 1;
            for (_, i, j) in pairs {
                if dsu.join(index[&anchors[i].0], index[&anchors[j].0]) {
                    airwires.push(Airwire {
                        net: net.to_owned(),
                        a: anchors[i].1,
                        b: anchors[j].1,
                    });
                    joined += 1;
                    if joined == islands.len() {
                        break;
                    }
                }
            }
        }
        airwires
    }
    fn describe(&self, owner: CopperRef, references: &BTreeMap<Uuid, String>) -> String {
        match owner {
            CopperRef::Pad { placement, pad } => {
                let placement = &self.placements[placement];
                format!(
                    "{} pad {}",
                    references
                        .get(&placement.component)
                        .map_or("?", String::as_str),
                    placement.footprint.pads[pad].number
                )
            }
            CopperRef::Track { track, .. } => format!(
                "track on {}",
                layer_name(self.tracks[track].layer, self.layer_count)
            ),
            CopperRef::Via(_) => "via".into(),
        }
    }
    /// [`Self::describe`] with the nets the copper carries: "track on F.Cu (SIG)".
    fn describe_on(&self, owner: CopperRef, nets: &BTreeSet<String>, references: &BTreeMap<Uuid, String>) -> String {
        let what = self.describe(owner, references);
        if nets.is_empty() {
            what
        } else {
            format!("{what} ({})", nets.iter().cloned().collect::<Vec<_>>().join(", "))
        }
    }
    /// Design rule check against exact copper geometry.
    pub fn drc(&self, netlist: &Netlist) -> Vec<Violation> {
        let conn = self.connectivity(netlist);
        let references: BTreeMap<Uuid, String> = netlist
            .nets
            .iter()
            .flat_map(|n| &n.pins)
            .map(|p| (p.component_id, p.reference.clone()))
            .collect();
        let object = |owner: CopperRef| match owner {
            CopperRef::Pad { placement, pad } => (0, placement, pad),
            CopperRef::Track { track, .. } => (1, track, 0),
            CopperRef::Via(v) => (2, v, 0),
        };
        let mut violations = vec![];
        for (island, nets) in conn.island_nets.iter().enumerate() {
            if nets.len() > 1 {
                let at = conn
                    .islands
                    .iter()
                    .position(|&i| i == island)
                    .map_or_else(Point::default, |i| conn.items[i].shape.center());
                violations.push(Violation {
                    kind: ViolationKind::Short,
                    message: format!(
                        "Short circuit between {}",
                        nets.iter().cloned().collect::<Vec<_>>().join(", ")
                    ),
                    at,
                });
            }
        }
        // Each pair keeps the larger of its two nets' class clearances, so the
        // prefilter must reach as far as the largest any class asks for.
        let nets_of = |island: usize| conn.island_nets[island].iter().map(String::as_str);
        let mut reported = BTreeSet::new();
        near_pairs(&conn.items, self.rules.max_clearance(), |i, j| {
            let (island_i, island_j) = (conn.islands[i], conn.islands[j]);
            if island_i == island_j {
                return;
            }
            if let (Some(a), Some(b)) = (conn.island_net(island_i), conn.island_net(island_j))
                && a == b
            {
                return;
            }
            let (a, b) = (&conn.items[i], &conn.items[j]);
            let (required, class) = self.rules.clearance_between_nets(nets_of(island_i), nets_of(island_j));
            let clearance = f64::from(required);
            let gap = a.shape.distance(&b.shape);
            let key = (object(a.owner), object(b.owner));
            if gap + 0.5 < clearance && reported.insert(key) {
                let (ca, cb) = (a.shape.center(), b.shape.center());
                violations.push(Violation {
                    kind: ViolationKind::Clearance,
                    message: format!(
                        "{} to {}: {:.3} mm gap, {} clearance {:.3} mm required",
                        self.describe_on(a.owner, &conn.island_nets[island_i], &references),
                        self.describe_on(b.owner, &conn.island_nets[island_j], &references),
                        gap / 1000.,
                        class.name,
                        clearance / 1000.
                    ),
                    at: Point::new(
                        ((i64::from(ca.x) + i64::from(cb.x)) / 2) as i32,
                        ((i64::from(ca.y) + i64::from(cb.y)) / 2) as i32,
                    ),
                });
            }
        });
        // A zone's fill is copper like any other: it keeps the clearance from every
        // island it is not part of that does not carry its own net, and from the
        // fill of any other zone of another net on its layer.
        let edge_clearance = f64::from(self.rules.edge_clearance);
        for (z, zone) in self.zones.iter().enumerate() {
            let label = zones::describe(zone, self.layer_count);
            let mut reported = BTreeSet::new();
            for (k, piece) in zone.fill.iter().enumerate() {
                let island = conn.zone_islands[z][k];
                let (min, max) = piece.bounds();
                for (i, item) in conn.items.iter().enumerate() {
                    if !(item.layers.0 <= zone.layer && zone.layer <= item.layers.1)
                        || conn.islands[i] == island
                    {
                        continue;
                    }
                    if conn.island_net(conn.islands[i]) == Some(zone.net.as_str())
                        && conn.island_net(island) == Some(zone.net.as_str())
                    {
                        continue;
                    }
                    let (lo, hi) = item.shape.bounds();
                    let slack = self.rules.max_clearance();
                    if lo.x > max.x + slack
                        || hi.x < min.x - slack
                        || lo.y > max.y + slack
                        || hi.y < min.y - slack
                    {
                        continue;
                    }
                    let (required, class) =
                        self.rules.clearance_between_nets([zone.net.as_str()], nets_of(conn.islands[i]));
                    let clearance = f64::from(required);
                    let gap = piece.distance(&item.shape);
                    if gap + 0.5 < clearance && reported.insert(object(item.owner)) {
                        violations.push(Violation {
                            kind: ViolationKind::Clearance,
                            message: format!(
                                "{label} to {}: {:.3} mm gap, {} clearance {:.3} mm required",
                                self.describe_on(item.owner, &conn.island_nets[conn.islands[i]], &references),
                                gap / 1000.,
                                class.name,
                                clearance / 1000.
                            ),
                            at: item.shape.center(),
                        });
                    }
                }
                for (other_z, other) in self.zones.iter().enumerate().skip(z + 1) {
                    if other.layer != zone.layer || other.net == zone.net {
                        continue;
                    }
                    for (m, other_piece) in other.fill.iter().enumerate() {
                        if conn.zone_islands[other_z][m] == island {
                            continue;
                        }
                        let (required, class) =
                            self.rules.clearance_between(Some(zone.net.as_str()), Some(other.net.as_str()));
                        let clearance = f64::from(required);
                        let gap = piece.distance_to_piece(other_piece);
                        if gap + 0.5 < clearance {
                            violations.push(Violation {
                                kind: ViolationKind::Clearance,
                                message: format!(
                                    "{label} to {}: {:.3} mm gap, {} clearance {:.3} mm required",
                                    zones::describe(other, self.layer_count),
                                    gap / 1000.,
                                    class.name,
                                    clearance / 1000.
                                ),
                                at: other_piece.outer[0],
                            });
                        }
                    }
                }
                let edge_gap = piece
                    .edges()
                    .flat_map(|(a, b)| {
                        let edge = Shape::segment(a, b, 0);
                        self.outline_edges()
                            .map(move |(c, d)| edge.distance(&Shape::segment(c, d, 0)))
                    })
                    .fold(f64::INFINITY, f64::min);
                if !self.outline_contains(piece.outer[0]) || edge_gap + 0.5 < edge_clearance {
                    violations.push(Violation {
                        kind: ViolationKind::BoardEdge,
                        message: format!(
                            "{label} is outside the board or within {:.3} mm of its edge",
                            edge_clearance / 1000.
                        ),
                        at: piece.outer[0],
                    });
                }
            }
        }
        let mut reported = BTreeSet::new();
        for item in &conn.items {
            let center = item.shape.center();
            let gap = self
                .outline_edges()
                .map(|(a, b)| item.shape.distance(&Shape::segment(a, b, 0)))
                .fold(f64::INFINITY, f64::min);
            if (!self.outline_contains(center) || gap + 0.5 < edge_clearance)
                && reported.insert(object(item.owner))
            {
                violations.push(Violation {
                    kind: ViolationKind::BoardEdge,
                    message: format!(
                        "{} is outside the board or within {:.3} mm of its edge",
                        self.describe(item.owner, &references),
                        edge_clearance / 1000.
                    ),
                    at: center,
                });
            }
        }
        // Every track at least as wide as its net asks for; an island shorting two
        // nets is already a finding, and asks for its widest.
        let mut reported = BTreeSet::new();
        for (i, item) in conn.items.iter().enumerate() {
            let CopperRef::Track { track, .. } = item.owner else { continue };
            let t = &self.tracks[track];
            let nets = &conn.island_nets[conn.islands[i]];
            let (required, rule) = match conn.island_net(conn.islands[i]) {
                Some(net) if self.rules.net_widths.contains_key(net) => {
                    (self.rules.width_for(net), format!("{net}'s width"))
                }
                _ => {
                    let class = self.rules.class_of_nets(nets.iter().map(String::as_str));
                    let widest = nets
                        .iter()
                        .map(|n| self.rules.width_for(n))
                        .max()
                        .unwrap_or(class.track_width);
                    (widest, format!("{} width", class.name))
                }
            };
            if t.width < required && reported.insert(track) {
                violations.push(Violation {
                    kind: ViolationKind::TrackWidth,
                    message: format!(
                        "track on {}{}: {:.3} mm wide, {rule} {:.3} mm required",
                        layer_name(t.layer, self.layer_count),
                        if nets.is_empty() {
                            String::new()
                        } else {
                            format!(" ({})", nets.iter().cloned().collect::<Vec<_>>().join(", "))
                        },
                        f64::from(t.width) / 1000.,
                        f64::from(required) / 1000.
                    ),
                    at: t.points.first().copied().unwrap_or_default(),
                });
            }
        }
        for (i, a) in self.placements.iter().enumerate() {
            let (a0, a1) = a.courtyard();
            for b in &self.placements[i + 1..] {
                let (b0, b1) = b.courtyard();
                if a.bottom == b.bottom && a0.x < b1.x && b0.x < a1.x && a0.y < b1.y && b0.y < a1.y
                {
                    violations.push(Violation {
                        kind: ViolationKind::Courtyard,
                        message: format!(
                            "Courtyards of {} and {} overlap",
                            references.get(&a.component).map_or("?", String::as_str),
                            references.get(&b.component).map_or("?", String::as_str)
                        ),
                        at: Point::new((a.at.x + b.at.x) / 2, (a.at.y + b.at.y) / 2),
                    });
                }
            }
        }
        for net in netlist.nets.iter().filter(|n| n.pins.len() > 1) {
            for pin in &net.pins {
                if let Some(placement) = self.placement(pin.component_id)
                    && !placement
                        .footprint
                        .pads
                        .iter()
                        .any(|p| p.number == pin.number)
                {
                    violations.push(Violation {
                        kind: ViolationKind::MissingPad,
                        message: format!(
                            "{} pin {} (net {}) has no pad in footprint {}",
                            pin.reference, pin.number, net.name, placement.footprint.name
                        ),
                        at: placement.at,
                    });
                }
            }
        }
        violations.extend(self.stitching_findings(netlist));
        for wire in self.ratsnest(&conn) {
            violations.push(Violation {
                kind: ViolationKind::Unrouted,
                message: format!("Unrouted connection in net {}", wire.net),
                at: Point::new((wire.a.x + wire.b.x) / 2, (wire.a.y + wire.b.y) / 2),
            });
        }
        violations.sort_by(|a, b| (a.kind, &a.message, a.at).cmp(&(b.kind, &b.message, b.at)));
        violations
    }
    pub fn validate(&self, components: &BTreeSet<Uuid>) -> Result<(), String> {
        let valid = |p: Point| p.x.abs_diff(0) <= 100_000_000 && p.y.abs_diff(0) <= 100_000_000;
        let size = |v: i32| (1..=100_000_000).contains(&v);
        if !(1..=32).contains(&self.layer_count) {
            return Err("Boards need between 1 and 32 copper layers".into());
        }
        if self.outline.len() < 3 || self.outline.iter().any(|p| !valid(*p)) {
            return Err("Board outline needs at least three valid points".into());
        }
        let r = &self.rules;
        if !size(r.track_width)
            || !(0..=100_000_000).contains(&r.clearance)
            || !(0..=100_000_000).contains(&r.edge_clearance)
            || !size(r.via_drill)
            || r.via_diameter <= r.via_drill
            || !size(r.via_diameter)
            || !(10..=100_000).contains(&r.routing_grid)
            || r.net_widths.values().any(|w| !size(*w))
            || !(0..=10_000).contains(&r.mask_expansion)
        {
            return Err("Invalid design rules".into());
        }
        r.check_net_classes()?;
        let mut placed = BTreeSet::new();
        for p in &self.placements {
            if !components.contains(&p.component) {
                return Err("Footprint refers to a missing component".into());
            }
            if !placed.insert(p.component) {
                return Err("Component has more than one footprint".into());
            }
            if !valid(p.at) || p.rotation > 3 {
                return Err("Invalid footprint placement".into());
            }
            for pad in &p.footprint.pads {
                if !valid(pad.at)
                    || !size(pad.size.x)
                    || !size(pad.size.y)
                    || pad
                        .drill
                        .is_some_and(|d| !size(d) || d > pad.size.x.max(pad.size.y))
                {
                    return Err(format!("Invalid pad in footprint {}", p.footprint.name));
                }
            }
            if p.footprint.silk.iter().flatten().any(|q| !valid(*q)) {
                return Err("Invalid footprint silkscreen".into());
            }
        }
        for t in &self.tracks {
            if t.layer >= self.layer_count
                || !size(t.width)
                || t.points.len() < 2
                || t.points.iter().any(|q| !valid(*q))
                || t.points.windows(2).any(|s| s[0] == s[1])
            {
                return Err("Tracks need a valid layer, width, and nonzero segments".into());
            }
        }
        for v in &self.vias {
            if !valid(v.at) || !size(v.drill) || v.diameter <= v.drill {
                return Err("Invalid via".into());
            }
        }
        for zone in &self.zones {
            zone.validate(self.layer_count)?;
        }
        Ok(())
    }
}

/// Result of bringing the board in line with the schematic.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct BoardSync {
    pub added: usize,
    pub removed: usize,
    /// The references of components that need a footprint but have no pads, in
    /// natural order. Their part carries none, so they are not on the board.
    pub without_pads: Vec<String>,
    /// The outline's size, width and height in micrometres, before and after the
    /// pass, when the pass grew it: parts added to a board with no free room left
    /// for them are packed below it and the outline follows them down.
    pub outline_grew: Option<(Point, Point)>,
}

/// Explicit native pads always belong on the board. Pin-bearing symbols without
/// pads are reported as missing pads; standalone power symbols and flags are not.
pub fn needs_footprint(component: &crate::Component) -> bool {
    component.pads.is_some() || (component.symbol.power_net.is_none()
        && !component.reference.starts_with('#')
        && !component.symbol.pins.is_empty())
}

impl Document {
    /// Place each new component with its part's pads and drop the placements of
    /// removed components. Existing placements are kept. A component with no pads is
    /// left off the board and listed in [`BoardSync::without_pads`]: pads come with
    /// the part, from a KiCad import or the pads editor, never from a generator.
    pub fn sync_board(&mut self) -> BoardSync {
        let wanted: BTreeSet<Uuid> = self
            .components
            .iter()
            .filter(|c| needs_footprint(c))
            .map(|c| c.id)
            .collect();
        let before = self.board.placements.len();
        self.board
            .placements
            .retain(|p| wanted.contains(&p.component));
        let removed = before - self.board.placements.len();
        let was_empty = self.board.placements.is_empty();
        let mut added = vec![];
        let mut without_pads = vec![];
        for c in self.components.iter().filter(|c| wanted.contains(&c.id)) {
            if self.board.placement(c.id).is_some() {
                continue;
            }
            let Some(pads) = &c.pads else {
                without_pads.push(c.reference.clone());
                continue;
            };
            self.board.placements.push(Placement {
                component: c.id,
                footprint: pads.clone(),
                at: Point::default(),
                rotation: 0,
                bottom: false,
            });
            added.push(c.id);
        }
        without_pads.sort_by(|a, b| footprint::natural_cmp(a, b));
        let size = |(min, max): (Point, Point)| Point::new(max.x - min.x, max.y - min.y);
        let before = size(self.board.outline_bounds());
        if was_empty {
            self.auto_place();
        } else if !added.is_empty() {
            // Into the room the board already has first, beside the parts each shares
            // a net with. Packing every addition as a new row under the lowest part
            // grew the board by a row per part: six parts added one at a time took a
            // stock 60 × 40 mm board to 60 × 54 before it was ever looked at, and one
            // capacitor grew a fitted 40 × 19 mm board to 40 × 21 (the eCAD re-audit's
            // B10 and item 16).
            let homeless = self.seat_in_free_room(&added);
            if !homeless.is_empty() {
                let (min, _) = self.board.outline_bounds();
                let top = self
                    .board
                    .placements
                    .iter()
                    .filter(|p| !homeless.contains(&p.component))
                    .map(|p| p.courtyard().1.y + PLACEMENT_SPACING)
                    .max()
                    .unwrap_or(min.y + PLACEMENT_SPACING);
                self.pack(&homeless, top);
            }
        }
        let after = size(self.board.outline_bounds());
        BoardSync {
            added: added.len(),
            removed,
            without_pads,
            outline_grew: (!was_empty && after != before).then_some((before, after)),
        }
    }
    /// Lay every part out afresh by its nets: the part with the most pins in the
    /// middle of the board, then each part, in [`Self::placement_order`], in the free
    /// room nearest the parts already laid out that it shares a small net with. Each
    /// keeps its side and its rotation. Where the room runs out the rest are packed
    /// in rows below, as [`Self::auto_place`] packs them, and the outline grows.
    ///
    /// [`Self::auto_place`] packs the same order into rows, so a part's neighbours
    /// were the ones beside it in the row and the next row began at the left edge
    /// however far that was from them (the eCAD re-audit's B10).
    pub fn place_by_nets(&mut self) {
        let order = self.placement_order();
        let homeless = self.seat_in_free_room(&order);
        if !homeless.is_empty() {
            let (min, _) = self.board.outline_bounds();
            let top = self
                .board
                .placements
                .iter()
                .filter(|p| !homeless.contains(&p.component))
                .map(|p| p.courtyard().1.y + PLACEMENT_SPACING)
                .max()
                .unwrap_or(min.y + PLACEMENT_SPACING);
            self.pack(&homeless, top);
        }
    }
    /// Seat each of `ids`, in turn, in the free room inside the outline nearest the
    /// placed parts it shares a small net with (a rail of more than eight parts is
    /// everyone's neighbour and so no one's), or nearest the placed parts' middle
    /// when it shares none. Free room is a spot where its courtyard keeps
    /// [`SEAT_GAP`] from the outline and from every other courtyard on its side, and
    /// the design rules' clearance from every piece of copper already on the board,
    /// so a part added to a routed board does not land on a track. Each keeps its
    /// side and its rotation. Returns the ones with no room anywhere, left where
    /// they were.
    fn seat_in_free_room(&mut self, ids: &[Uuid]) -> Vec<Uuid> {
        let mut pending: BTreeSet<Uuid> = ids.iter().copied().collect();
        let mut nets_of: BTreeMap<Uuid, Vec<usize>> = BTreeMap::new();
        for (n, net) in self.netlist().nets.iter().enumerate() {
            let members: BTreeSet<Uuid> = net.pins.iter().map(|p| p.component_id).collect();
            if members.len() <= 8 {
                for m in members {
                    nets_of.entry(m).or_default().push(n);
                }
            }
        }
        let (min, max) = self.board.outline_bounds();
        let clearance = f64::from(self.board.rules.max_clearance());
        let gap = f64::from(SEAT_GAP);
        let mut homeless = vec![];
        for &id in ids {
            let Some(index) = self.board.placements.iter().position(|p| p.component == id) else {
                continue;
            };
            let seated: Vec<&Placement> = self
                .board
                .placements
                .iter()
                .filter(|p| !pending.contains(&p.component))
                .collect();
            let mine = nets_of.get(&id).cloned().unwrap_or_default();
            let near: Vec<Point> = seated
                .iter()
                .filter(|p| {
                    nets_of
                        .get(&p.component)
                        .is_some_and(|theirs| theirs.iter().any(|n| mine.contains(n)))
                })
                .map(|p| p.at)
                .collect();
            let centre = |points: &[Point]| {
                let n = points.len().max(1) as i64;
                let (x, y) = points
                    .iter()
                    .fold((0i64, 0i64), |(x, y), p| (x + i64::from(p.x), y + i64::from(p.y)));
                Point::new((x / n) as i32, (y / n) as i32)
            };
            let target = if !near.is_empty() {
                centre(&near)
            } else if !seated.is_empty() {
                centre(&seated.iter().map(|p| p.at).collect::<Vec<_>>())
            } else {
                Point::new((min.x + max.x) / 2, (min.y + max.y) / 2)
            };
            let mut placement = self.board.placements[index].clone();
            placement.at = Point::default();
            let courtyards: Vec<Shape> = seated
                .iter()
                .filter(|p| p.bottom == placement.bottom)
                .map(|p| {
                    let (min, max) = p.courtyard();
                    Shape::Rect { min, max }
                })
                .collect();
            let copper: Vec<Shape> = self
                .board
                .copper()
                .into_iter()
                .filter(|item| match item.owner {
                    CopperRef::Pad { placement, .. } => {
                        !pending.contains(&self.board.placements[placement].component)
                    }
                    _ => true,
                })
                .map(|item| item.shape)
                .collect();
            let (c0, c1) = placement.courtyard();
            let step = 500;
            let span = |lo: i32, hi: i32, c_lo: i32, c_hi: i32| {
                let first = snap_up(lo + SEAT_GAP - c_lo, step);
                let last = snap_down(hi - SEAT_GAP - c_hi, step);
                (first..=last).step_by(step as usize)
            };
            let mut spots: Vec<Point> = span(min.x, max.x, c0.x, c1.x)
                .flat_map(|x| span(min.y, max.y, c0.y, c1.y).map(move |y| Point::new(x, y)))
                .collect();
            let middle = Point::new((c0.x + c1.x) / 2, (c0.y + c1.y) / 2);
            let far = |at: &Point| {
                let (dx, dy) = (
                    i64::from(at.x + middle.x - target.x),
                    i64::from(at.y + middle.y - target.y),
                );
                (dx * dx + dy * dy, at.y, at.x)
            };
            spots.sort_by_key(far);
            let free = spots.into_iter().find(|&at| {
                let body = Shape::Rect { min: c0.offset(at), max: c1.offset(at) };
                let corners = [
                    c0.offset(at),
                    c1.offset(at),
                    Point::new(c0.x, c1.y).offset(at),
                    Point::new(c1.x, c0.y).offset(at),
                ];
                corners.iter().all(|&p| self.board.outline_contains(p))
                    && self
                        .board
                        .outline_edges()
                        .all(|(a, b)| body.distance(&Shape::segment(a, b, 0)) >= gap)
                    && courtyards.iter().all(|c| body.distance(c) >= gap)
                    && copper.iter().all(|c| body.distance(c) >= clearance)
            });
            match free {
                Some(at) => {
                    placement.at = at;
                    self.board.placements[index] = placement;
                    pending.remove(&id);
                }
                None => homeless.push(id),
            }
        }
        homeless
    }
    /// Pack all placements in rows inside the outline, keeping connected parts together.
    pub fn auto_place(&mut self) {
        let order = self.placement_order();
        let (min, _) = self.board.outline_bounds();
        self.pack(&order, min.y + PLACEMENT_SPACING);
        let (min, max) = self.board.outline_bounds();
        let corners: Vec<Point> = self
            .board
            .placements
            .iter()
            .flat_map(|p| {
                let (a, b) = p.courtyard();
                [a, b]
            })
            .collect();
        if let Some((a, b)) = bounds_of(&corners) {
            let centre = |lo: i32, hi: i32, a: i32, b: i32| {
                let shift = (lo + hi) / 2 - (a + b) / 2;
                shift.div_euclid(100) * 100
            };
            let shift = Point::new(
                centre(min.x, max.x, a.x, b.x),
                centre(min.y, max.y, a.y, b.y),
            );
            for p in &mut self.board.placements {
                p.at = p.at.offset(shift);
            }
        }
    }
    /// Components ordered by breadth-first traversal of small shared nets.
    fn placement_order(&self) -> Vec<Uuid> {
        let placed: BTreeSet<Uuid> = self.board.placements.iter().map(|p| p.component).collect();
        let mut neighbours: BTreeMap<Uuid, BTreeSet<Uuid>> = BTreeMap::new();
        for net in self.netlist().nets {
            let members: BTreeSet<Uuid> = net
                .pins
                .iter()
                .map(|p| p.component_id)
                .filter(|c| placed.contains(c))
                .collect();
            if members.len() > 8 {
                continue;
            }
            for a in &members {
                neighbours
                    .entry(*a)
                    .or_default()
                    .extend(members.iter().filter(|b| *b != a));
            }
        }
        let mut seeds: Vec<&crate::Component> = self
            .components
            .iter()
            .filter(|c| placed.contains(&c.id))
            .collect();
        seeds.sort_by(|a, b| {
            b.symbol
                .pins
                .len()
                .cmp(&a.symbol.pins.len())
                .then_with(|| footprint::natural_cmp(&a.reference, &b.reference))
        });
        let reference: BTreeMap<Uuid, &str> = self
            .components
            .iter()
            .map(|c| (c.id, c.reference.as_str()))
            .collect();
        let mut order = vec![];
        let mut seen = BTreeSet::new();
        for seed in seeds {
            if !seen.insert(seed.id) {
                continue;
            }
            let mut queue = std::collections::VecDeque::from([seed.id]);
            while let Some(id) = queue.pop_front() {
                order.push(id);
                let mut next: Vec<Uuid> = neighbours
                    .get(&id)
                    .into_iter()
                    .flatten()
                    .filter(|n| !seen.contains(*n))
                    .copied()
                    .collect();
                next.sort_by(|a, b| footprint::natural_cmp(reference[a], reference[b]));
                for n in next {
                    seen.insert(n);
                    queue.push_back(n);
                }
            }
        }
        order
    }
    /// Row-pack the given placements starting at `top`, growing a rectangular outline
    /// downward if they do not fit.
    fn pack(&mut self, ids: &[Uuid], top: i32) {
        let (min, max) = self.board.outline_bounds();
        let left = min.x + PLACEMENT_SPACING;
        let right = max.x - PLACEMENT_SPACING;
        let mut cursor = Point::new(left, top);
        let mut row_height = 0;
        for id in ids {
            let Some(p) = self
                .board
                .placements
                .iter_mut()
                .find(|p| p.component == *id)
            else {
                continue;
            };
            p.at = Point::default();
            let (c0, c1) = p.courtyard();
            let (w, h) = (c1.x - c0.x, c1.y - c0.y);
            if cursor.x + w > right && cursor.x > left {
                cursor = Point::new(left, cursor.y + row_height + PLACEMENT_SPACING);
                row_height = 0;
            }
            p.at = Point::new(snap_up(cursor.x - c0.x, 100), snap_up(cursor.y - c0.y, 100));
            cursor.x += w + PLACEMENT_SPACING;
            row_height = row_height.max(h);
        }
        let bottom = cursor.y + row_height + PLACEMENT_SPACING;
        if bottom > max.y && self.board.outline_is_rectangle() {
            self.board.outline = rectangle(min, Point::new(max.x, snap_up(bottom, 1000)));
        }
    }
    /// Resize a rectangular outline around all courtyards with a margin.
    pub fn fit_board_outline(&mut self, margin: i32) {
        let corners: Vec<Point> = self
            .board
            .placements
            .iter()
            .flat_map(|p| {
                let (a, b) = p.courtyard();
                [a, b]
            })
            .chain(self.board.tracks.iter().flat_map(|t| t.points.clone()))
            .chain(self.board.vias.iter().map(|v| v.at))
            .collect();
        if let Some((min, max)) = bounds_of(&corners) {
            self.board.outline = rectangle(
                Point::new(
                    snap_down(min.x - margin, 1000),
                    snap_down(min.y - margin, 1000),
                ),
                Point::new(snap_up(max.x + margin, 1000), snap_up(max.y + margin, 1000)),
            );
        }
    }
    /// The schematic net each drawn track carries, for the tracks whose copper island
    /// carries exactly one. A track joined to nothing, or bridging two nets, has none.
    pub fn track_nets(&self) -> BTreeMap<Uuid, String> {
        let connectivity = self.board.connectivity(&self.netlist());
        let mut nets = BTreeMap::new();
        for (i, item) in connectivity.items.iter().enumerate() {
            if let CopperRef::Track { track, .. } = item.owner
                && let Some(net) = connectivity.island_net(connectivity.islands[i])
            {
                nets.insert(self.board.tracks[track].id, net.to_owned());
            }
        }
        nets
    }
    /// Give every drawn track the width its net asks for, or the default width where its
    /// net has no rule of its own. Tracks whose width was set by hand keep it. Returns
    /// how many tracks changed. Widening copper can break clearances, so run the design
    /// rule check afterwards.
    pub fn apply_net_widths(&mut self) -> usize {
        let nets = self.track_nets();
        let rules = self.board.rules.clone();
        let mut changed = 0;
        for track in self.board.tracks.iter_mut().filter(|t| !t.fixed_width) {
            let width = nets
                .get(&track.id)
                .map_or(rules.track_width, |net| rules.width_for(net));
            if track.width != width {
                track.width = width;
                changed += 1;
            }
        }
        changed
    }
    /// Set one track's width, whatever its net asks for. [`Document::apply_net_widths`]
    /// then leaves this track alone until [`Document::follow_net_width`] gives it back.
    pub fn set_track_width(&mut self, id: Uuid, width: i32) -> Result<(), String> {
        if !(1..=100_000_000).contains(&width) {
            return Err("A track needs a positive width".into());
        }
        let track = self
            .board
            .tracks
            .iter_mut()
            .find(|t| t.id == id)
            .ok_or("Track no longer exists")?;
        track.width = width;
        track.fixed_width = true;
        Ok(())
    }
    /// Hand one track back to its net's width rule.
    pub fn follow_net_width(&mut self, id: Uuid) {
        let width = self
            .track_nets()
            .get(&id)
            .map_or(self.board.rules.track_width, |net| {
                self.board.rules.width_for(net)
            });
        if let Some(track) = self.board.tracks.iter_mut().find(|t| t.id == id) {
            track.width = width;
            track.fixed_width = false;
        }
    }
    pub fn board_drc(&self) -> Vec<Violation> {
        self.board.drc(&self.netlist())
    }
}
fn snap_up(v: i32, step: i32) -> i32 {
    v.div_euclid(step) * step + if v.rem_euclid(step) == 0 { 0 } else { step }
}
fn snap_down(v: i32, step: i32) -> i32 {
    v.div_euclid(step) * step
}
