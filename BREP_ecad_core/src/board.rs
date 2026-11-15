//! Printed circuit board layout: footprints, copper, connectivity, and design checks.
//!
//! Board coordinates use the schematic's integer micrometres with positive Y down.
//! Copper connectivity is derived purely from geometry and net identity comes from
//! the schematic netlist, so the schematic remains the single source of truth.
use crate::graphic::{GraphicLayer, LayerGraphic};
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
    /// A rectangle with rounded corners, of [`Pad::corner_radius`].
    RoundRect,
    /// A rectangle tapered by [`Pad::trapezoid_delta`] (KiCad's `rect_delta`):
    /// one pair of opposite sides lengthened and the other shortened.
    Trapezoid,
    /// Any simple polygon, [`Pad::polygon`], around the pad's position, its
    /// outline rounded by [`Pad::corner_radius`].
    Custom,
}
/// The most corners a custom pad's polygon may have.
pub const MAX_PAD_CORNERS: usize = 1000;

/// What a pad's PRESET (its kind, side and shape) would put on the solder mask and
/// paste layers, overridden where the user says otherwise. The default is "as the
/// preset says", and serializes as nothing, so a pad with no overrides is stored
/// exactly as it was before overrides existed.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct PadLayers {
    /// Solder mask stays OVER the pad on the footprint's front (tented): no opening.
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub cover_front: bool,
    /// Solder mask stays over the pad on the footprint's back.
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub cover_back: bool,
    /// No solder paste aperture for this surface-mount pad.
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub no_paste: bool,
    /// The mask opening's growth beyond the copper, replacing the board's
    /// [`DesignRules::mask_expansion`] for this pad.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub mask_margin: Option<i32>,
    /// The paste aperture's growth beyond the copper (negative shrinks it), in
    /// place of the default 0: the aperture matches the copper.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub paste_margin: Option<i32>,
    /// The paste split into a grid of smaller windows (a windowpane), as a large
    /// thermal pad's is so the part does not float on one big deposit.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub paste_windows: Option<PasteWindows>,
}
impl PadLayers {
    pub fn is_auto(&self) -> bool {
        *self == Self::default()
    }
}
/// A windowpane paste stencil: the aperture's bounding box cut into `columns`
/// by `rows` equal windows `gap` apart, each kept only where it lies wholly on
/// the aperture.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct PasteWindows {
    pub columns: u8,
    pub rows: u8,
    /// The web of stencil left between windows.
    pub gap: i32,
}
impl PasteWindows {
    /// Why these windows cannot be cut, if they cannot.
    pub fn problem(&self) -> Option<&'static str> {
        if !(1..=20).contains(&self.columns) || !(1..=20).contains(&self.rows) {
            Some("needs between 1 and 20 paste windows each way")
        } else if !(0..=10_000).contains(&self.gap) {
            Some("needs a paste window gap between 0 and 10 mm")
        } else {
            None
        }
    }
    /// The windows cut from `aperture`.
    pub fn cut(&self, aperture: &Shape) -> Vec<Shape> {
        let (min, max) = aperture.bounds();
        let (columns, rows) = (i64::from(self.columns.max(1)), i64::from(self.rows.max(1)));
        let gap = i64::from(self.gap.max(0));
        let span = |lo: i32, hi: i32, n: i64, k: i64| {
            let (lo, hi) = (i64::from(lo), i64::from(hi));
            let cell = (hi - lo - gap * (n - 1)) as f64 / n as f64;
            let start = lo as f64 + k as f64 * (cell + gap as f64);
            (start.round() as i32, (start + cell).round() as i32)
        };
        let mut out = vec![];
        for row in 0..rows {
            let (y0, y1) = span(min.y, max.y, rows, row);
            for column in 0..columns {
                let (x0, x1) = span(min.x, max.x, columns, column);
                if x1 > x0 && y1 > y0 {
                    let (lo, hi) = (Point::new(x0, y0), Point::new(x1, y1));
                    if aperture.covers_box(lo, hi) {
                        out.push(Shape::Rect { min: lo, max: hi });
                    }
                }
            }
        }
        out
    }
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
    /// Through-hole drill diameter; `None` for surface-mount pads. For an OVAL
    /// (slotted) hole, its width along the pad's x ([`Pad::drill_height`]).
    #[serde(default)]
    pub drill: Option<i32>,
    /// An oval (slotted) hole's height along the pad's y, KiCad's
    /// `(drill oval <width> <height>)`; `None` for a round hole of [`Pad::drill`].
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub drill_height: Option<i32>,
    /// Whether a drilled hole is plated. Unplated holes go to the NPTH drill file.
    #[serde(default = "plated_by_default")]
    pub plated: bool,
    /// Corner radius of a [`PadShape::RoundRect`] pad, at most half its smaller side.
    #[serde(default, skip_serializing_if = "is_zero")]
    pub corner_radius: i32,
    /// A surface-mount pad on the footprint's BACK copper (B.Cu) rather than its
    /// front. A part mounted on the bottom flips this with the rest of the part.
    /// Ignored for a drilled pad, which spans every copper layer.
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub back: bool,
    /// Mask and paste overrides of what the pad's preset generates.
    #[serde(default, skip_serializing_if = "PadLayers::is_auto")]
    pub layers: PadLayers,
    /// A [`PadShape::Trapezoid`] pad's taper, KiCad's `rect_delta`: `x`
    /// lengthens the left side and shortens the right by as much, `y`
    /// lengthens the bottom and shortens the top.
    #[serde(default, skip_serializing_if = "is_origin")]
    pub trapezoid_delta: Point,
    /// A [`PadShape::Custom`] pad's copper: a closed polygon of corners relative
    /// to [`Pad::at`], which it covers.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub polygon: Vec<Point>,
}
fn plated_by_default() -> bool {
    true
}
fn is_zero(v: &i32) -> bool {
    *v == 0
}
fn is_origin(p: &Point) -> bool {
    *p == Point::default()
}
impl Pad {
    /// A pad of the given number, place, size and shape with every other property
    /// at its default: surface mount, front, no overrides.
    pub fn smd(number: impl Into<String>, at: Point, size: Point, shape: PadShape) -> Self {
        Self {
            number: number.into(),
            at,
            size,
            shape,
            drill: None,
            drill_height: None,
            plated: true,
            corner_radius: 0,
            back: false,
            layers: PadLayers::default(),
            trapezoid_delta: Point::default(),
            polygon: vec![],
        }
    }
    /// The corner radius actually drawn: the stored one for a rounded rectangle,
    /// clamped to half the smaller side, a custom pad's rounding, else 0.
    pub fn effective_corner_radius(&self) -> i32 {
        match self.shape {
            PadShape::RoundRect => self.corner_radius.clamp(0, self.size.x.min(self.size.y) / 2),
            PadShape::Custom => self.corner_radius.clamp(0, 100_000),
            _ => 0,
        }
    }
    /// A trapezoid or custom pad's corners in footprint coordinates, else `None`.
    pub fn corners(&self) -> Option<Vec<Point>> {
        let at = |x: i64, y: i64| {
            let c = |v: i64| v.clamp(i64::from(i32::MIN), i64::from(i32::MAX)) as i32;
            Point::new(c(i64::from(self.at.x) + x), c(i64::from(self.at.y) + y))
        };
        match self.shape {
            PadShape::Trapezoid => {
                let (hx, hy) = (i64::from(self.size.x) / 2, i64::from(self.size.y) / 2);
                let (dx, dy) = (i64::from(self.trapezoid_delta.x) / 2, i64::from(self.trapezoid_delta.y) / 2);
                Some(vec![
                    at(-hx - dy, hy + dx),
                    at(hx + dy, hy - dx),
                    at(hx - dy, -hy + dx),
                    at(-hx + dy, -hy - dx),
                ])
            }
            PadShape::Custom => Some(self.polygon.iter().map(|p| at(i64::from(p.x), i64::from(p.y))).collect()),
            _ => None,
        }
    }
    /// The pad's copper in footprint coordinates.
    pub fn local_shape(&self) -> Shape {
        match self.corners() {
            Some(corners) => Shape::polygon(corners, self.effective_corner_radius()),
            None => Shape::rounded_pad(self.at, self.size, self.shape, self.effective_corner_radius()),
        }
    }
    /// Why a trapezoid's taper or a custom pad's polygon is not copper, if it is
    /// not.
    pub fn outline_problem(&self) -> Option<String> {
        let far = |v: i32| !(-100_000_000..=100_000_000).contains(&v);
        match self.shape {
            PadShape::Trapezoid => {
                let d = self.trapezoid_delta;
                if d.x.unsigned_abs() >= self.size.y.unsigned_abs() || d.y.unsigned_abs() >= self.size.x.unsigned_abs() {
                    return Some("needs a taper shorter than the side it narrows".into());
                }
                if d.x != 0 && d.y != 0 {
                    return Some("tapers one pair of sides only".into());
                }
                None
            }
            PadShape::Custom => {
                let n = self.polygon.len();
                if n < 3 {
                    return Some("needs at least three corners".into());
                }
                if n > MAX_PAD_CORNERS {
                    return Some(format!("has more than {MAX_PAD_CORNERS} corners"));
                }
                if self.polygon.iter().any(|p| far(p.x) || far(p.y)) || far(self.at.x) || far(self.at.y) {
                    return Some("lies outside the drawing area".into());
                }
                if ring_area2(&self.polygon) == 0 {
                    return Some("encloses no area".into());
                }
                if ring_crosses_itself(&self.polygon) {
                    return Some("crosses itself".into());
                }
                if polygon_depth(&self.polygon, Point::default()) + f64::from(self.effective_corner_radius()) < 0. {
                    return Some("does not cover its own position".into());
                }
                None
            }
            _ => None,
        }
    }
    /// Which sides of the FOOTPRINT the pad's copper reaches: (front, back).
    pub fn copper_sides(&self) -> (bool, bool) {
        match self.drill {
            Some(_) => (true, true),
            None => (!self.back, self.back),
        }
    }
    /// Whether the pad gets a solder mask opening on the footprint's front (or back).
    pub fn mask_open(&self, front: bool) -> bool {
        let (f, b) = self.copper_sides();
        if front {
            f && !self.layers.cover_front
        } else {
            b && !self.layers.cover_back
        }
    }
    /// Whether the pad gets a paste aperture on the footprint's front (or back).
    pub fn paste_open(&self, front: bool) -> bool {
        self.drill.is_none() && !self.layers.no_paste && self.back != front
    }
    /// The hole's extent, width (x) and height (y) in footprint coordinates: a
    /// round hole's diameter both ways, a slot's own two. `None` with no drill.
    pub fn hole_size(&self) -> Option<Point> {
        let width = self.drill?;
        Some(Point::new(width, self.drill_height.unwrap_or(width)))
    }
    /// The hole in footprint coordinates: a circle, or a slot's stadium.
    pub fn local_hole(&self) -> Option<Shape> {
        Some(Shape::pad(self.at, self.hole_size()?, PadShape::Oval))
    }
    /// The plated copper ring left round the drill, at its narrowest. `None` for
    /// a pad with no drill or an unplated hole; negative where the hole breaks
    /// out of the copper.
    pub fn annular_ring(&self) -> Option<i32> {
        let drill = self.drill.filter(|_| self.plated)?;
        if self.drill_height.is_none() && self.corners().is_none() {
            // A round hole: half the pad's smaller side less the drill's radius,
            // as it has always been measured.
            return Some((self.size.x.min(self.size.y) - drill) / 2);
        }
        Some(self.hole_clearance()?.floor() as i32)
    }
    /// How far the hole stays inside the pad's copper at its nearest: the hole is
    /// a disc swept along a segment. The preset shapes are convex, so the depth
    /// of the copper over the segment is least at one of its ends. A polygon may
    /// turn inward. A hole within the polygon's own corners, crossing none of its
    /// edges, has the nearest edge's distance and the rounding as its ring; any
    /// other (in the rounding, or across a notch) is measured along its length.
    fn hole_clearance(&self) -> Option<f64> {
        let Shape::Capsule { a, b, radius } = self.local_hole()? else {
            return None;
        };
        let copper = self.local_shape();
        if let Shape::Polygon { points, radius: round } = &copper {
            let within = polygon_contains(points, a)
                && polygon_contains(points, b)
                && ring_edges(points).all(|(p, q)| !segments_intersect(p, q, a, b));
            if !within {
                // Through the rounding, or across a notch: the copper's own depth,
                // sampled along the hole's segment at most a micrometre apart and
                // taken half a step shallower, as depth changes no faster than
                // the point moves.
                let len = f64::from(b.x - a.x).hypot(f64::from(b.y - a.y));
                let steps = (len.ceil() as usize).clamp(1, 4096);
                let step = len / steps as f64;
                let depth = (0..=steps)
                    .map(|k| {
                        let t = k as f64 / steps as f64;
                        let p = Point::new(
                            (f64::from(a.x) + t * f64::from(b.x - a.x)).round() as i32,
                            (f64::from(a.y) + t * f64::from(b.y - a.y)).round() as i32,
                        );
                        copper.depth(p)
                    })
                    .fold(f64::INFINITY, f64::min);
                return Some(depth - step / 2. - f64::from(radius));
            }
            let edges = ring_edges(points).map(|(p, q)| segment_distance(p, q, a, b)).fold(f64::INFINITY, f64::min);
            return Some(edges + f64::from(*round) - f64::from(radius));
        }
        Some(copper.depth(a).min(copper.depth(b)) - f64::from(radius))
    }
    /// Why the hole does not fit the pad, if it does not: a drill must be
    /// positive and no wider or taller than the pad.
    fn hole_problem(&self) -> Option<&'static str> {
        let hole = self.hole_size()?;
        let far = |v: i32| !(-100_000_000..=100_000_000).contains(&v);
        if [self.at.x, self.at.y, self.size.x, self.size.y, hole.x, hole.y].into_iter().any(far) {
            // Measured nowhere: the geometry would overflow before it could say why.
            return Some("lies outside the drawing area");
        }
        if hole.x <= 0 || hole.y <= 0 {
            Some("needs a positive drill")
        } else if self.drill_height.is_some() && self.hole_clearance().is_some_and(|c| c < 0.) {
            Some("has a slot larger than the pad")
        } else if self.drill_height.is_none() && self.corners().is_some() && self.hole_clearance().is_some_and(|c| c < 0.) {
            Some("has a drill larger than the pad")
        } else if self.drill_height.is_none() && hole.x > self.size.x.max(self.size.y) {
            Some("has a drill larger than the pad")
        } else {
            None
        }
    }
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
    /// Courtyard clearance round the pads and silkscreen, in place of
    /// [`COURTYARD_MARGIN`].
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub courtyard_margin: Option<i32>,
    /// Drawn items on the silkscreen, fabrication and courtyard layers of either
    /// side, beside the front silkscreen polylines in `silk`.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub graphics: Vec<LayerGraphic>,
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
            if let Some(problem) = pad.outline_problem() {
                return Err(format!("{name} {problem}."));
            }
            if let Some(problem) = pad.hole_problem() {
                return Err(format!("{name} {problem}."));
            }
            if !(0..=100_000).contains(&pad.corner_radius) {
                return Err(format!("{name} needs a corner radius between 0 and 100 mm."));
            }
            let margins = [pad.layers.mask_margin, pad.layers.paste_margin];
            if margins.into_iter().flatten().any(|m| !(-10_000..=10_000).contains(&m)) {
                return Err(format!("{name} has a mask or paste margin beyond 10 mm."));
            }
            if let Some(problem) = pad.layers.paste_windows.and_then(|w| w.problem()) {
                return Err(format!("{name} {problem}."));
            }
        }
        if self.courtyard_margin.is_some_and(|m| !(0..=100_000).contains(&m)) {
            return Err("The courtyard clearance must be between 0 and 100 mm.".into());
        }
        if self.silk.iter().any(|line| line.len() < 2) {
            return Err("Silkscreen lines need at least two points.".into());
        }
        if self.graphics.len() > 100_000 {
            return Err("A footprint can carry at most 100000 drawn items.".into());
        }
        for (i, graphic) in self.graphics.iter().enumerate() {
            if let Some(problem) = graphic.problem() {
                return Err(format!("Drawing {} on {} {problem}.", i + 1, graphic.layer.name()));
            }
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
    /// Every drawn item as (layer, pen width, filled, centre lines) in footprint
    /// coordinates: the `silk` polylines as front silkscreen at
    /// [`crate::fabrication::SILK_WIDTH`], then the graphics. Text on a back layer
    /// is written mirrored, so it reads from the back.
    pub fn drawing(&self) -> impl Iterator<Item = Drawn> + '_ {
        let silk = self.silk.iter().map(|line| Drawn {
            layer: GraphicLayer::FrontSilk,
            width: crate::fabrication::SILK_WIDTH,
            filled: false,
            strokes: vec![line.clone()],
        });
        silk.chain(self.graphics.iter().map(|g| Drawn {
            layer: g.layer,
            width: g.width(),
            filled: g.filled,
            strokes: g.strokes(!g.layer.front()),
        }))
    }
    /// The bounds of the courtyard drawn on the footprint's front (F.CrtYd,
    /// `front`) or back (B.CrtYd), if it has one. The front one replaces the
    /// rectangle generated round pads and legend.
    pub fn drawn_courtyard(&self, front: bool) -> Option<(Point, Point)> {
        let layer = if front { GraphicLayer::FrontCourtyard } else { GraphicLayer::BackCourtyard };
        let points: Vec<Point> = self
            .graphics
            .iter()
            .filter(|g| g.layer == layer)
            .flat_map(LayerGraphic::points)
            .collect();
        bounds_of(&points)
    }
    /// Bounds of pads and silkscreen (on either side) in footprint coordinates.
    pub fn bounds(&self) -> (Point, Point) {
        let mut points: Vec<Point> = self
            .pads
            .iter()
            .flat_map(|p| {
                let (min, max) = match p.corners() {
                    Some(_) => p.local_shape().bounds(),
                    None => Shape::pad(p.at, p.size, PadShape::Rect).bounds(),
                };
                [min, max]
            })
            .collect();
        points.extend(self.silk.iter().flatten().copied());
        points.extend(
            self.graphics
                .iter()
                .filter(|g| g.layer.is_silk())
                .flat_map(LayerGraphic::points),
        );
        bounds_of(&points).unwrap_or_default()
    }
}

/// One drawn item of a footprint, read by [`Footprint::drawing`].
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Drawn {
    pub layer: GraphicLayer,
    pub width: i32,
    pub filled: bool,
    /// Centre lines in footprint coordinates.
    pub strokes: Vec<Vec<Point>>,
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
        match pad.corners() {
            Some(corners) => Shape::polygon(
                corners.into_iter().map(|p| self.transform(p)).collect(),
                pad.effective_corner_radius(),
            ),
            None => Shape::rounded_pad(self.transform(pad.at), size, pad.shape, pad.effective_corner_radius()),
        }
    }
    /// A surface-mount pad's paste on the board: its copper grown by its paste
    /// margin, cut into windows when it has a windowpane. The windows are cut in
    /// the footprint's own frame, so columns and rows turn with the part.
    pub fn pad_paste(&self, pad: &Pad) -> Vec<Shape> {
        let margin = pad.layers.paste_margin.unwrap_or(0);
        let Some(windows) = pad.layers.paste_windows else {
            return self.pad_shape(pad).grown_parts(margin);
        };
        pad.local_shape()
            .grown_parts(margin)
            .iter()
            .flat_map(|aperture| windows.cut(aperture))
            .filter_map(|window| {
                let Shape::Rect { min, max } = window else { return None };
                let (a, b) = (self.transform(min), self.transform(max));
                Some(Shape::Rect {
                    min: Point::new(a.x.min(b.x), a.y.min(b.y)),
                    max: Point::new(a.x.max(b.x), a.y.max(b.y)),
                })
            })
            .collect()
    }
    /// A drilled pad's hole on the board: a circle, or a slot turned with the part.
    pub fn pad_hole(&self, pad: &Pad) -> Option<Shape> {
        let hole = pad.hole_size()?;
        let hole = if self.rotation % 2 == 1 { Point::new(hole.y, hole.x) } else { hole };
        Some(Shape::pad(self.transform(pad.at), hole, PadShape::Oval))
    }
    /// Inclusive copper layer range of a pad: a drilled pad spans every layer, a
    /// surface-mount pad sits on the board side its footprint side faces.
    pub fn pad_layers(&self, pad: &Pad, layer_count: u8) -> (u8, u8) {
        let last = layer_count.saturating_sub(1);
        match (pad.drill, self.bottom != pad.back) {
            (Some(_), _) => (0, last),
            (None, false) => (0, 0),
            (None, true) => (last, last),
        }
    }
    /// Whether a surface-mount pad of this placement sits on the board's BOTTOM
    /// copper: a back pad of a top part, or a front pad of a bottom part.
    pub fn has_back_copper_pad(&self) -> bool {
        self.footprint.pads.iter().any(|pad| pad.drill.is_none() && pad.back != self.bottom)
    }
    /// The footprint side that faces the board's top (true) or bottom (false):
    /// true means the footprint's front.
    pub fn front_faces(&self, board_top: bool) -> bool {
        board_top != self.bottom
    }
    /// Every drawn item of the footprint that lands on `layer` of the BOARD, as
    /// (pen width, filled, centre lines) in board coordinates: a part on the
    /// bottom puts its front layers on the board's back.
    pub fn drawing_on(&self, layer: GraphicLayer) -> Vec<(i32, bool, Vec<Vec<Point>>)> {
        self.footprint
            .drawing()
            .filter(|d| if self.bottom { d.layer.flipped() } else { d.layer } == layer)
            .map(|d| {
                let strokes = d
                    .strokes
                    .into_iter()
                    .map(|line| line.into_iter().map(|p| self.transform(p)).collect())
                    .collect();
                (d.width, d.filled, strokes)
            })
            .collect()
    }
    /// The board layer one of the footprint's graphics lands on, and its centre
    /// lines in board coordinates.
    pub fn place_graphic(&self, graphic: &LayerGraphic) -> (GraphicLayer, Vec<Vec<Point>>) {
        let layer = if self.bottom { graphic.layer.flipped() } else { graphic.layer };
        let strokes = graphic
            .strokes(!graphic.layer.front())
            .into_iter()
            .map(|line| line.into_iter().map(|p| self.transform(p)).collect())
            .collect();
        (layer, strokes)
    }
    /// The world-space rectangle round a courtyard drawn on the footprint's
    /// BACK (B.CrtYd), which is on the board side opposite the one the part is
    /// mounted on.
    pub fn back_courtyard(&self) -> Option<(Point, Point)> {
        let (min, max) = self.footprint.drawn_courtyard(false)?;
        let corners = [min, max, Point::new(min.x, max.y), Point::new(max.x, min.y)]
            .map(|p| self.transform(p));
        bounds_of(&corners)
    }
    /// The courtyard rectangles this part occupies on the board's top (`true`)
    /// or bottom side: its own on the side it is mounted on, and a drawn back
    /// courtyard on the other.
    pub fn courtyard_on(&self, top: bool) -> Option<(Point, Point)> {
        if self.bottom != top { Some(self.courtyard()) } else { self.back_courtyard() }
    }
    /// World-space courtyard rectangle: round the drawn courtyard when the
    /// footprint has one, otherwise round the pads and legend plus the clearance.
    pub fn courtyard(&self) -> (Point, Point) {
        let drawn = self.footprint.drawn_courtyard(true);
        let (min, max) = drawn.unwrap_or_else(|| self.footprint.bounds());
        let corners = [min, max, Point::new(min.x, max.y), Point::new(max.x, min.y)]
            .map(|p| self.transform(p));
        let (min, max) = bounds_of(&corners).unwrap_or_default();
        let margin = match drawn {
            Some(_) => 0,
            None => self.footprint.courtyard_margin.unwrap_or(COURTYARD_MARGIN),
        };
        (
            Point::new(min.x - margin, min.y - margin),
            Point::new(max.x + margin, max.y + margin),
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
    /// The copper layers a BLIND or BURIED via joins, inclusive and top first
    /// (`from < to`). `None` is a through via, joining every layer, and is how
    /// every via was stored before spans existed.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub span: Option<(u8, u8)>,
}
/// What a via joins: every layer, an outer layer to an inner one, or inner
/// layers only.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum ViaKind {
    Through,
    /// From the top or bottom copper to an inner layer.
    Blind,
    /// Between inner layers only.
    Buried,
}
impl Via {
    /// A through via.
    pub fn through(at: Point, diameter: i32, drill: i32) -> Self {
        Self { id: Uuid::new_v4(), at, diameter, drill, span: None }
    }
    /// The inclusive copper layer range the via joins on a board of
    /// `layer_count` layers.
    pub fn layers(&self, layer_count: u8) -> (u8, u8) {
        let last = layer_count.saturating_sub(1);
        match self.span {
            Some((from, to)) => (from.min(last), to.min(last)),
            None => (0, last),
        }
    }
    pub fn kind(&self, layer_count: u8) -> ViaKind {
        let last = layer_count.saturating_sub(1);
        match self.layers(layer_count) {
            (0, to) if to == last => ViaKind::Through,
            (from, to) if from == 0 || to == last => ViaKind::Blind,
            _ => ViaKind::Buried,
        }
    }
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
    /// The narrowest plated copper ring a drilled pad or via may keep round its
    /// hole. Written only when it differs from the default, so a board that never
    /// changed it is stored as before.
    #[serde(
        default = "default_min_annular_ring",
        skip_serializing_if = "is_default_min_annular_ring"
    )]
    pub min_annular_ring: i32,
    /// Whether the solder mask covers every via (tented, the default) or opens
    /// over each via's ring on both sides. Written only when off.
    #[serde(default = "tented_by_default", skip_serializing_if = "Clone::clone")]
    pub tent_vias: bool,
}
fn tented_by_default() -> bool {
    true
}
fn default_mask_expansion() -> i32 {
    50
}
/// The default minimum annular ring, 0.13 mm: the ring most board houses quote
/// for their standard process (KiCad's own default is 0.1 mm).
pub const DEFAULT_MIN_ANNULAR_RING: i32 = 130;
fn default_min_annular_ring() -> i32 {
    DEFAULT_MIN_ANNULAR_RING
}
fn is_default_min_annular_ring(v: &i32) -> bool {
    *v == DEFAULT_MIN_ANNULAR_RING
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
            min_annular_ring: DEFAULT_MIN_ANNULAR_RING,
            tent_vias: true,
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
    /// Lines, shapes and text drawn on the board itself, on a silkscreen or
    /// fabrication layer, belonging to no part: a board's name and revision, a
    /// connector's legend, an outline on its assembly drawing. Written only when
    /// there are some, as zones are.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub graphics: Vec<BoardGraphic>,
    /// Holes milled through the board inside its outline (KiCad's inner
    /// Edge.Cuts loops): closed polygons, each wholly inside the outline and
    /// apart from every other. Copper keeps the edge clearance from them as it
    /// does from the outline. Written only when there are some, as zones are.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub cutouts: Vec<BoardCutout>,
}
/// The most corners one cutout may have.
pub const MAX_CUTOUT_CORNERS: usize = 1000;
/// The most cutouts one board may have.
pub const MAX_CUTOUTS: usize = 1000;
/// A hole milled through the board: a closed polygon (its last corner joins
/// its first), with the identity a selection and an undo step hold it by.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct BoardCutout {
    pub id: Uuid,
    pub points: Vec<Point>,
}
impl BoardCutout {
    pub fn new(points: Vec<Point>) -> Self {
        Self { id: Uuid::new_v4(), points }
    }
}
/// The edges of a closed ring, edge `i` running from corner `i` to the next.
fn ring_edges(ring: &[Point]) -> impl Iterator<Item = (Point, Point)> + '_ {
    let n = ring.len();
    (0..n).map(move |i| (ring[i], ring[(i + 1) % n]))
}
/// True when two edges of `ring` that do not share a corner touch.
fn ring_crosses_itself(ring: &[Point]) -> bool {
    let n = ring.len();
    (0..n).any(|i| {
        (i + 2..n).any(|j| {
            (i, j) != (0, n - 1)
                && segments_intersect(ring[i], ring[(i + 1) % n], ring[j], ring[(j + 1) % n])
        })
    })
}
/// True when an edge of `a` touches an edge of `b`.
fn rings_touch(a: &[Point], b: &[Point]) -> bool {
    ring_edges(a).any(|(p, q)| ring_edges(b).any(|(r, s)| segments_intersect(p, q, r, s)))
}
/// A drawn item on the board itself: a [`LayerGraphic`] with the identity a
/// selection and an undo step hold it by.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct BoardGraphic {
    pub id: Uuid,
    #[serde(flatten)]
    pub graphic: LayerGraphic,
}
impl BoardGraphic {
    pub fn new(graphic: LayerGraphic) -> Self {
        Self { id: Uuid::new_v4(), graphic }
    }
    /// Why this item cannot be on a board, if it cannot: what any drawn item
    /// is refused for ([`LayerGraphic::problem`]), and a courtyard, which is the
    /// room a PART needs and means nothing on the board by itself.
    pub fn problem(&self) -> Option<String> {
        if self.graphic.layer.is_courtyard() {
            return Some(format!(
                "is on {}: a courtyard belongs to a part's footprint, not to the board",
                self.graphic.layer.name()
            ));
        }
        self.graphic.problem().map(str::to_owned)
    }
}
impl Board {
    /// Every item drawn on the board ITSELF on `layer`, as (pen width, filled,
    /// centre lines) in board coordinates — the board-level counterpart of
    /// [`Placement::drawing_on`]. Back-layer text is written mirrored, so it reads
    /// from the board's back.
    pub fn drawing_on(&self, layer: GraphicLayer) -> Vec<(i32, bool, Vec<Vec<Point>>)> {
        self.graphics
            .iter()
            .filter(|g| g.graphic.layer == layer)
            .map(|g| (g.graphic.width(), g.graphic.filled, g.graphic.strokes(!layer.front())))
            .collect()
    }
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
            graphics: vec![],
            cutouts: vec![],
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

/// The box grown by `grow` on every side; a shrink stops at a 1-unit box about
/// the middle and never turns the box inside out. Growing by 0 is the identity.
fn grow_box(min: Point, max: Point, grow: i32) -> (Point, Point) {
    if grow >= 0 {
        return (
            Point::new(min.x - grow, min.y - grow),
            Point::new(max.x + grow, max.y + grow),
        );
    }
    let shrink = |lo: i32, hi: i32| {
        let (a, b) = (lo - grow, hi + grow);
        if a < b {
            (a, b)
        } else {
            let mid = (i64::from(lo) + i64::from(hi)).div_euclid(2) as i32;
            (mid, mid + 1)
        }
    };
    let (x0, x1) = shrink(min.x, max.x);
    let (y0, y1) = shrink(min.y, max.y);
    (Point::new(x0, y0), Point::new(x1, y1))
}
pub fn rectangle(min: Point, max: Point) -> Vec<Point> {
    vec![min, Point::new(max.x, min.y), max, Point::new(min.x, max.y)]
}

/// A copper primitive for clearance and connectivity calculations.
#[derive(Clone, Debug, PartialEq)]
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
    /// A rectangle with corners rounded by `radius`: the rectangle `min..max`
    /// shrunk by `radius` on every side, swept by a disc of `radius`.
    RoundRect {
        min: Point,
        max: Point,
        radius: i32,
    },
    /// A simple polygon swept by a disc of `radius`: a trapezoid or custom
    /// pad's copper (rounded by its corner radius), and its mask opening grown
    /// from it. `points` run counter-clockwise in a y-up frame, as
    /// [`Shape::polygon`] puts them.
    Polygon {
        points: Vec<Point>,
        radius: i32,
    },
}
impl Shape {
    /// A polygon swept by a disc of `radius`, its corners turned to run
    /// counter-clockwise in a y-up frame (a positive signed area in board
    /// coordinates) whichever way they were given, so a mirrored part's pad
    /// has the same winding as any other.
    pub fn polygon(mut points: Vec<Point>, radius: i32) -> Self {
        if ring_area2(&points) < 0 {
            points.reverse();
        }
        Self::Polygon { points, radius: radius.max(0) }
    }
    /// A point the shape surely covers, or touches: for a containment test.
    fn anchor(&self) -> Point {
        match self {
            Self::Rect { min, .. } => *min,
            Self::RoundRect { min, max, radius } => Self::core(*min, *max, *radius).0,
            Self::Capsule { a, .. } => *a,
            Self::Polygon { points, .. } => points.first().copied().unwrap_or_default(),
        }
    }
    /// Whether the shape covers the whole box `min..max`.
    pub fn covers_box(&self, min: Point, max: Point) -> bool {
        let corners = rectangle(min, max);
        if !corners.iter().all(|c| self.depth(*c) >= 0.) {
            return false;
        }
        // The preset shapes are convex: covering the corners covers the box. A
        // polygon may turn inward between them, so none of its outline's edges
        // may enter the box.
        let Self::Polygon { .. } = self else {
            return true;
        };
        let ring = self.outline(8);
        let (lo, hi) = (Point::new(min.x + 1, min.y + 1), Point::new(max.x - 1, max.y - 1));
        lo.x > hi.x || lo.y > hi.y || ring_edges(&ring).all(|(p, q)| segment_rect_distance(p, q, lo, hi) > 0.)
    }
    /// A pad's copper, a rounded rectangle taken square-cornered
    /// ([`Shape::rounded_pad`] gives it its corners).
    pub fn pad(center: Point, size: Point, shape: PadShape) -> Self {
        Self::rounded_pad(center, size, shape, 0)
    }
    /// A pad's copper. `corner` is the corner radius of a rounded rectangle (0
    /// makes it an ordinary rectangle); other shapes ignore it.
    pub fn rounded_pad(center: Point, size: Point, shape: PadShape, corner: i32) -> Self {
        let half = Point::new(size.x / 2, size.y / 2);
        match shape {
            PadShape::RoundRect if corner > 0 => {
                let min = Point::new(center.x - half.x, center.y - half.y);
                Self::RoundRect {
                    min,
                    max: Point::new(min.x + size.x, min.y + size.y),
                    radius: corner.min(size.x.min(size.y) / 2),
                }
            }
            // A trapezoid or custom pad given only its size: its bounding box
            // ([`Pad::local_shape`] gives it its corners).
            PadShape::Rect | PadShape::RoundRect | PadShape::Trapezoid | PadShape::Custom => {
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
    /// The shape grown by `grow` on every side (shrunk when negative): a pad's
    /// solder mask opening or paste aperture. A shrink never inverts the shape.
    pub fn grown(&self, grow: i32) -> Self {
        if grow == 0 {
            return self.clone();
        }
        if let Self::Polygon { points, radius } = self {
            let radius = radius + grow;
            if radius >= 0 {
                return Self::Polygon { points: points.clone(), radius };
            }
            // Shrunk past its rounding: the polygon itself moves in.
            return match shrink_ring(points, -radius) {
                Some(points) => Self::Polygon { points, radius: 0 },
                None => Self::circle(polygon_inner_point(points), 1),
            };
        }
        match *self {
            Self::Rect { min, max } => {
                let (min, max) = grow_box(min, max, grow);
                Self::Rect { min, max }
            }
            Self::RoundRect { min, max, radius } => {
                let (min, max) = grow_box(min, max, grow);
                let radius = (radius + grow).clamp(0, (max.x - min.x).min(max.y - min.y) / 2);
                // Shrunk past its corners, it is a plain rectangle.
                if radius == 0 { Self::Rect { min, max } } else { Self::RoundRect { min, max, radius } }
            }
            Self::Capsule { a, b, radius } => Self::Capsule {
                a,
                b,
                radius: (radius + grow).max(1),
            },
            Self::Polygon { .. } => unreachable!(),
        }
    }
    /// A rounded polygon's outline as exact arcs and the straights between
    /// them, for a host that draws curves (the STEP export). `None` for any
    /// other shape, an unrounded polygon (its corners are its outline), or one
    /// whose rounding fills a notch, where only [`Shape::outline`] is exact.
    pub fn swept_corners(&self) -> Option<Vec<SweptCorner>> {
        let Self::Polygon { points, radius } = self else { return None };
        if *radius == 0 {
            return None;
        }
        let (raw, concave) = swept_outline(points, *radius, 8);
        if concave && ring_crosses_itself(&raw) {
            return None;
        }
        Some(swept_corners_of(points, *radius))
    }
    /// The shape grown by `grow`, as [`Shape::grown`], but a polygon that a
    /// shrink cuts in two keeps every piece (two lands joined by a thin bridge
    /// each keep their paste); a shrink that leaves nothing gives none.
    pub fn grown_parts(&self, grow: i32) -> Vec<Self> {
        match self {
            Self::Polygon { points, radius } if radius + grow < 0 => shrink_parts(points, -(radius + grow))
                .into_iter()
                .map(|points| Self::Polygon { points, radius: 0 })
                .collect(),
            _ => vec![self.grown(grow)],
        }
    }
    /// The rectangle a rounded rectangle's disc sweeps: `min..max` less `radius`.
    fn core(min: Point, max: Point, radius: i32) -> (Point, Point) {
        (
            Point::new(min.x + radius, min.y + radius),
            Point::new(max.x - radius, max.y - radius),
        )
    }
    /// The outline as a closed polygon, counter-clockwise in a y-up frame, arcs
    /// faceted `facets` to the quarter turn, each vertex ON the true outline.
    pub fn outline(&self, facets: usize) -> Vec<Point> {
        let arc = |out: &mut Vec<Point>, c: Point, r: i32, from: f64, steps: usize, sweep: f64| {
            for i in 0..=steps {
                let t = from + sweep * i as f64 / steps.max(1) as f64;
                out.push(Point::new(
                    c.x + (f64::from(r) * t.cos()).round() as i32,
                    c.y + (f64::from(r) * t.sin()).round() as i32,
                ));
            }
        };
        let quarter = std::f64::consts::FRAC_PI_2;
        let facets = facets.max(1);
        if let Self::Polygon { points, radius } = self {
            if *radius == 0 {
                return points.clone();
            }
            let (out, concave) = swept_outline(points, *radius, facets);
            // Where the rounding fills a notch, the swept edges of the notch
            // cross: the loop they make lies inside the copper and is cut out.
            return if concave { remove_loops(out) } else { out };
        }
        match *self {
            Self::Rect { min, max } => rectangle(min, max),
            Self::RoundRect { min, max, radius } => {
                let (lo, hi) = Self::core(min, max, radius);
                let mut out = vec![];
                for (k, c) in [Point::new(hi.x, lo.y), hi, Point::new(lo.x, hi.y), lo]
                    .into_iter()
                    .enumerate()
                {
                    arc(&mut out, c, radius, quarter * (k as f64 - 1.), facets, quarter);
                }
                out.dedup();
                if out.len() > 1 && out.first() == out.last() {
                    out.pop();
                }
                out
            }
            Self::Capsule { a, b, radius } => {
                let (dx, dy) = (f64::from(b.x - a.x), f64::from(b.y - a.y));
                let base = if dx == 0. && dy == 0. { 0. } else { dy.atan2(dx) } - quarter;
                let mut out = vec![];
                arc(&mut out, b, radius, base, 2 * facets, 2. * quarter);
                if a == b {
                    out.pop();
                    arc(&mut out, a, radius, base + 2. * quarter, 2 * facets, 2. * quarter);
                    out.pop();
                } else {
                    arc(&mut out, a, radius, base + 2. * quarter, 2 * facets, 2. * quarter);
                }
                out.dedup();
                if out.len() > 1 && out.first() == out.last() {
                    out.pop();
                }
                out
            }
            Self::Polygon { .. } => unreachable!(),
        }
    }
    pub fn bounds(&self) -> (Point, Point) {
        if let Self::Polygon { points, radius } = self {
            let (min, max) = bounds_of(points).unwrap_or_default();
            return (
                Point::new(min.x.saturating_sub(*radius), min.y.saturating_sub(*radius)),
                Point::new(max.x.saturating_add(*radius), max.y.saturating_add(*radius)),
            );
        }
        match *self {
            Self::Rect { min, max } | Self::RoundRect { min, max, .. } => (min, max),
            Self::Capsule { a, b, radius } => (
                Point::new(a.x.min(b.x) - radius, a.y.min(b.y) - radius),
                Point::new(a.x.max(b.x) + radius, a.y.max(b.y) + radius),
            ),
            Self::Polygon { .. } => unreachable!(),
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
        if let Self::Polygon { points, radius } = self {
            return (-polygon_depth(points, p) - f64::from(*radius)).max(0.);
        }
        match *self {
            Self::Rect { min, max } => point_rect_distance(p, min, max),
            Self::RoundRect { min, max, radius } => {
                let (lo, hi) = Self::core(min, max, radius);
                (point_rect_distance(p, lo, hi) - f64::from(radius)).max(0.)
            }
            Self::Capsule { a, b, radius } => {
                (point_segment_distance(p, a, b) - f64::from(radius)).max(0.)
            }
            Self::Polygon { .. } => unreachable!(),
        }
    }
    /// How deep `p` lies inside the shape: its distance to the boundary,
    /// positive inside and negative outside.
    pub fn depth(&self, p: Point) -> f64 {
        let inside_box = |min: Point, max: Point| {
            let d = [p.x - min.x, max.x - p.x, p.y - min.y, max.y - p.y];
            f64::from(d.into_iter().min().unwrap_or(0))
        };
        if let Self::Polygon { points, radius } = self {
            return polygon_depth(points, p) + f64::from(*radius);
        }
        match *self {
            Self::Rect { min, max } => {
                let inner = inside_box(min, max);
                if inner >= 0. { inner } else { -point_rect_distance(p, min, max) }
            }
            Self::RoundRect { min, max, radius } => {
                let (lo, hi) = Self::core(min, max, radius);
                let inner = inside_box(lo, hi);
                f64::from(radius) + if inner >= 0. { inner } else { -point_rect_distance(p, lo, hi) }
            }
            Self::Capsule { a, b, radius } => f64::from(radius) - point_segment_distance(p, a, b),
            Self::Polygon { .. } => unreachable!(),
        }
    }
    pub fn contains(&self, p: Point) -> bool {
        self.distance_to_point(p) <= 0.
    }
    /// Gap between two shapes; zero when they touch or overlap.
    pub fn distance(&self, other: &Self) -> f64 {
        // A polygon is its edges swept by its disc, unless one shape lies
        // wholly inside the other.
        if let Self::Polygon { points, radius } = self {
            if points.is_empty() {
                return f64::INFINITY;
            }
            if polygon_contains(points, other.anchor()) || other.contains(points[0]) {
                return 0.;
            }
            let edges = ring_edges(points)
                .map(|(a, b)| Self::Capsule { a, b, radius: 0 }.distance(other))
                .fold(f64::INFINITY, f64::min);
            return (edges - f64::from(*radius)).max(0.);
        }
        if let Self::Polygon { .. } = other {
            return other.distance(self);
        }
        // A rounded rectangle is its core rectangle swept by a disc: measure from
        // the core and take the radius off.
        if let Self::RoundRect { min, max, radius } = *self {
            let (lo, hi) = Self::core(min, max, radius);
            return (Self::Rect { min: lo, max: hi }.distance(other) - f64::from(radius)).max(0.);
        }
        if let Self::RoundRect { .. } = *other {
            return other.distance(self);
        }
        match (self.clone(), other.clone()) {
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
            (Self::RoundRect { .. } | Self::Polygon { .. }, _) | (_, Self::RoundRect { .. } | Self::Polygon { .. }) => {
                unreachable!()
            }
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
/// One corner of a rounded polygon's outline ([`Shape::swept_corners`]): an arc
/// of the disc about a corner that turns outward, from angle `from` (radians,
/// board coordinates) through `sweep`, or the point where the two swept edges
/// of a corner that turns inward meet. Straight edges join consecutive corners.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum SweptCorner {
    Arc { centre: Point, radius: i32, from: f64, sweep: f64 },
    Meet(f64, f64),
}
impl SweptCorner {
    /// Where the corner's piece of outline starts and ends.
    pub fn ends(&self) -> ((f64, f64), (f64, f64)) {
        match *self {
            Self::Arc { centre, radius, from, sweep } => {
                let at = |t: f64| (f64::from(centre.x) + f64::from(radius) * t.cos(), f64::from(centre.y) + f64::from(radius) * t.sin());
                (at(from), at(from + sweep))
            }
            Self::Meet(x, y) => ((x, y), (x, y)),
        }
    }
}
/// A counter-clockwise polygon's corners swept by a disc of `radius`.
fn swept_corners_of(points: &[Point], radius: i32) -> Vec<SweptCorner> {
    let n = points.len();
    let r = f64::from(radius);
    (0..n)
        .map(|i| {
            let (prev, here, next) = (points[(i + n - 1) % n], points[i], points[(i + 1) % n]);
            let (d0, d1) = (unit(prev, here), unit(here, next));
            // Outward normal of a counter-clockwise (y-up) edge.
            let (n0, n1) = ((d0.1, -d0.0), (d1.1, -d1.0));
            if d0.0 * d1.1 - d0.1 * d1.0 > 1e-12 {
                let from = n0.1.atan2(n0.0);
                let mut sweep = n1.1.atan2(n1.0) - from;
                while sweep <= 0. {
                    sweep += 2. * std::f64::consts::PI;
                }
                SweptCorner::Arc { centre: here, radius, from, sweep }
            } else {
                let (mx, my) = miter(n0, n1);
                SweptCorner::Meet(f64::from(here.x) + mx * r, f64::from(here.y) + my * r)
            }
        })
        .collect()
}
/// A rounded polygon's outline before any loop is cut, arcs faceted `facets`
/// to the quarter turn, and whether the polygon turns inward anywhere.
fn swept_outline(points: &[Point], radius: i32, facets: usize) -> (Vec<Point>, bool) {
    let quarter = std::f64::consts::FRAC_PI_2;
    let mut out = vec![];
    let round = |(x, y): (f64, f64)| Point::new(x.round() as i32, y.round() as i32);
    for corner in swept_corners_of(points, radius) {
        match corner {
            SweptCorner::Arc { centre, radius, from, sweep } => {
                let steps = ((sweep / quarter) * facets.max(1) as f64).ceil().max(1.) as usize;
                for k in 0..=steps {
                    let t = from + sweep * k as f64 / steps as f64;
                    out.push(round((
                        f64::from(centre.x) + f64::from(radius) * t.cos(),
                        f64::from(centre.y) + f64::from(radius) * t.sin(),
                    )));
                }
            }
            SweptCorner::Meet(x, y) => out.push(round((x, y))),
        }
    }
    out.dedup();
    if out.len() > 1 && out.first() == out.last() {
        out.pop();
    }
    let n = points.len();
    let concave = (0..n).any(|i| cross(points[(i + n - 1) % n], points[i], points[(i + 1) % n]) < 0);
    (out, concave)
}
/// Signed distance from `p` to a closed polygon's boundary: positive inside.
pub(crate) fn polygon_depth(polygon: &[Point], p: Point) -> f64 {
    let d = ring_edges(polygon)
        .map(|(a, b)| point_segment_distance(p, a, b))
        .fold(f64::INFINITY, f64::min);
    if polygon_contains(polygon, p) { d } else { -d }
}
/// Twice a ring's signed area: positive when it runs counter-clockwise in a
/// y-up frame.
fn ring_area2(ring: &[Point]) -> i128 {
    ring_edges(ring)
        .map(|(a, b)| i128::from(a.x) * i128::from(b.y) - i128::from(b.x) * i128::from(a.y))
        .sum()
}
/// The unit direction from `a` to `b`, or zero where they coincide.
fn unit(a: Point, b: Point) -> (f64, f64) {
    let (dx, dy) = (f64::from(b.x) - f64::from(a.x), f64::from(b.y) - f64::from(a.y));
    let len = dx.hypot(dy);
    if len == 0. { (0., 0.) } else { (dx / len, dy / len) }
}
/// Where two lines offset one unit along normals `n0` and `n1` meet, from the
/// corner they share; the normal itself when the lines are parallel.
fn miter(n0: (f64, f64), n1: (f64, f64)) -> (f64, f64) {
    let (sx, sy) = (n0.0 + n1.0, n0.1 + n1.1);
    let dot = 1. + n0.0 * n1.0 + n0.1 * n1.1;
    if dot < 1e-9 { n1 } else { (sx / dot, sy / dot) }
}
/// A counter-clockwise ring moved `by` inward along every edge, each corner
/// where two moved edges meet. An edge that the move turns round (a short
/// chamfer, say) is dropped and its neighbours meet instead, and a loop the
/// move leaves where the ring turns inward is cut out; there the corner stays
/// sharp, a little more than an exact inset. `None` when nothing is left or the
/// result would leave the ring's own inset.
fn shrink_parts(ring: &[Point], by: i32) -> Vec<Vec<Point>> {
    type Line = ((f64, f64), (f64, f64));
    let d = f64::from(by);
    // Where two moved edges meet: the second's start when they are one line.
    let meet = |((px, py), (ux, uy)): Line, ((qx, qy), (vx, vy)): Line| -> Option<(f64, f64)> {
        let turn = ux * vy - uy * vx;
        if turn.abs() < 1e-9 {
            return (ux * vx + uy * vy > 0.).then_some((qx, qy));
        }
        let t = ((qx - px) * vy - (qy - py) * vx) / turn;
        Some((px + t * ux, py + t * uy))
    };
    // Each edge as a line moved inward: a point on it and its direction.
    let mut lines: Vec<Line> = ring_edges(ring)
        .filter(|(a, b)| a != b)
        .map(|(a, b)| {
            let u = unit(a, b);
            ((f64::from(a.x) - u.1 * d, f64::from(a.y) + u.0 * d), u)
        })
        .collect();
    for _ in 0..=ring.len() {
        let n = lines.len();
        if n < 3 {
            return vec![];
        }
        let Some(corners) = (0..n).map(|i| meet(lines[(i + n - 1) % n], lines[i])).collect::<Option<Vec<(f64, f64)>>>() else {
            return vec![];
        };
        // How far each moved edge still runs along its own direction.
        let run = |i: usize| {
            let (s, e) = (corners[i], corners[(i + 1) % n]);
            (e.0 - s.0) * lines[i].1.0 + (e.1 - s.1) * lines[i].1.1
        };
        match (0..n).filter(|&i| run(i) <= 0.5).min_by(|&i, &j| run(i).total_cmp(&run(j))) {
            Some(worst) => {
                lines.remove(worst);
            }
            None => {
                let out: Vec<Point> = corners.iter().map(|c| Point::new(c.0.round() as i32, c.1.round() as i32)).collect();
                let mut parts = vec![];
                split_loops(out, &mut parts, &mut ring.len().max(8));
                parts.retain(|part| {
                    part.iter().all(|p| polygon_depth(ring, *p) >= d - 2.) && !ring_crosses_itself(part)
                });
                return parts;
            }
        }
    }
    vec![]
}
/// The largest piece [`shrink_parts`] leaves.
fn shrink_ring(ring: &[Point], by: i32) -> Option<Vec<Point>> {
    shrink_parts(ring, by).into_iter().max_by_key(|part| ring_area2(part))
}
/// A ring cut at each place it crosses itself into the loops it makes,
/// keeping each loop that runs counter-clockwise (a reversed loop is where an
/// inset overran itself and holds nothing); at most `budget` cuts.
fn split_loops(ring: Vec<Point>, out: &mut Vec<Vec<Point>>, budget: &mut usize) {
    let n = ring.len();
    let crossing = (0..n).find_map(|i| {
        let (a, b) = (ring[i], ring[(i + 1) % n]);
        (i + 2..n).find_map(|j| {
            let (c, d) = (ring[j], ring[(j + 1) % n]);
            ((i, j) != (0, n - 1) && segments_intersect(a, b, c, d)).then_some((i, j))
        })
    });
    match crossing {
        Some((i, j)) if *budget > 0 && n >= 4 => {
            *budget -= 1;
            let x = crossing_point(ring[i], ring[(i + 1) % n], ring[j], ring[(j + 1) % n]);
            let mut outer: Vec<Point> = ring[..=i].to_vec();
            outer.push(x);
            outer.extend_from_slice(&ring[j + 1..]);
            let mut inner = vec![x];
            inner.extend_from_slice(&ring[i + 1..=j]);
            for mut part in [outer, inner] {
                part.dedup();
                if part.len() > 1 && part.first() == part.last() {
                    part.pop();
                }
                if part.len() >= 3 {
                    split_loops(part, out, budget);
                }
            }
        }
        _ => {
            if n >= 3 && ring_area2(&ring) > 0 {
                out.push(ring);
            }
        }
    }
}
/// Where segments `a..b` and `c..d`, known to touch, meet.
fn crossing_point(a: Point, b: Point, c: Point, d: Point) -> Point {
    let (u, v) = ((f64::from(b.x - a.x), f64::from(b.y - a.y)), (f64::from(d.x - c.x), f64::from(d.y - c.y)));
    let turn = u.0 * v.1 - u.1 * v.0;
    if turn.abs() < 1e-9 {
        return if within_box(a, b, c) { c } else { a };
    }
    let t = (f64::from(c.x - a.x) * v.1 - f64::from(c.y - a.y) * v.0) / turn;
    Point::new((f64::from(a.x) + t * u.0).round() as i32, (f64::from(a.y) + t * u.1).round() as i32)
}
/// A ring with the loops it makes where it crosses itself cut out, keeping at
/// each crossing the side of larger counter-clockwise area: the outline of an
/// offset whose edges overran one another.
fn remove_loops(mut ring: Vec<Point>) -> Vec<Point> {
    'cut: for _ in 0..ring.len() {
        let n = ring.len();
        if n < 4 {
            break;
        }
        for i in 0..n {
            let (a, b) = (ring[i], ring[(i + 1) % n]);
            for j in i + 2..n {
                if (i, j) == (0, n - 1) {
                    continue;
                }
                let (c, d) = (ring[j], ring[(j + 1) % n]);
                if a.x.max(b.x) < c.x.min(d.x)
                    || c.x.max(d.x) < a.x.min(b.x)
                    || a.y.max(b.y) < c.y.min(d.y)
                    || c.y.max(d.y) < a.y.min(b.y)
                    || !segments_intersect(a, b, c, d)
                {
                    continue;
                }
                let x = crossing_point(a, b, c, d);
                let mut outer: Vec<Point> = ring[..=i].to_vec();
                outer.push(x);
                outer.extend_from_slice(&ring[j + 1..]);
                let mut inner = vec![x];
                inner.extend_from_slice(&ring[i + 1..=j]);
                outer.dedup();
                inner.dedup();
                ring = if ring_area2(&inner) > ring_area2(&outer) { inner } else { outer };
                continue 'cut;
            }
        }
        break;
    }
    ring
}
/// A point inside a simple polygon: the middle of its widest span across the
/// middle of its height.
pub fn polygon_inner_point(ring: &[Point]) -> Point {
    let Some((min, max)) = bounds_of(ring) else {
        return Point::default();
    };
    let y = (i64::from(min.y) + i64::from(max.y)) / 2;
    let mut xs: Vec<f64> = ring_edges(ring)
        .filter(|(a, b)| (i64::from(a.y) > y) != (i64::from(b.y) > y))
        .map(|(a, b)| {
            let t = (y as f64 - f64::from(a.y)) / (f64::from(b.y) - f64::from(a.y));
            f64::from(a.x) + t * (f64::from(b.x) - f64::from(a.x))
        })
        .collect();
    xs.sort_by(f64::total_cmp);
    let widest = xs.chunks(2).filter(|c| c.len() == 2).max_by(|a, b| (a[1] - a[0]).total_cmp(&(b[1] - b[0])));
    match widest {
        Some(c) => Point::new(((c[0] + c[1]) / 2.).round() as i32, y as i32),
        None => ring[0],
    }
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
    /// A plated hole whose copper ring is narrower than
    /// [`DesignRules::min_annular_ring`].
    AnnularRing,
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
    /// True when `p` is on the board: inside the outline (even-odd) and in no
    /// cutout.
    pub fn outline_contains(&self, p: Point) -> bool {
        polygon_contains(&self.outline, p) && !self.cutouts.iter().any(|c| polygon_contains(&c.points, p))
    }
    /// Every edge of the board: the outline's, then each cutout's. Copper keeps
    /// the edge clearance from all of them.
    pub fn outline_edges(&self) -> impl Iterator<Item = (Point, Point)> + '_ {
        std::iter::once(self.outline.as_slice())
            .chain(self.cutouts.iter().map(|c| c.points.as_slice()))
            .flat_map(ring_edges)
    }
    /// Why `points` cannot be a cutout of this board, if it cannot: fewer than
    /// three corners, no area, edges that cross, a corner off the board or on
    /// its outline, or touching another cutout. `except` is the index of the
    /// cutout being replaced, which is not checked against itself.
    pub fn cutout_problem(&self, points: &[Point], except: Option<usize>) -> Option<String> {
        self.cutout_problem_among(points, &self.cutouts, except)
    }
    /// [`Board::cutout_problem`] against `others` only.
    fn cutout_problem_among(&self, points: &[Point], others: &[BoardCutout], except: Option<usize>) -> Option<String> {
        if points.len() < 3 {
            return Some("needs at least three corners".into());
        }
        // The checks below compare every edge with every other: a cutout is a
        // milled shape, and a thousand corners is far more than one needs.
        if points.len() > MAX_CUTOUT_CORNERS {
            return Some(format!("has more than {MAX_CUTOUT_CORNERS} corners"));
        }
        if zones::ring_area(points) == 0. {
            return Some("encloses no area".into());
        }
        if ring_crosses_itself(points) {
            return Some("crosses itself".into());
        }
        if points.iter().any(|&p| !polygon_contains(&self.outline, p)) || rings_touch(points, &self.outline) {
            return Some("is not wholly inside the board outline".into());
        }
        let bounds = bounds_of(points);
        for (k, other) in others.iter().enumerate() {
            // Two cutouts whose boxes are apart can neither touch nor hold one
            // another: the edge-by-edge checks are only for the near ones.
            let apart = match (bounds, bounds_of(&other.points)) {
                (Some((a0, a1)), Some((b0, b1))) => a1.x < b0.x || b1.x < a0.x || a1.y < b0.y || b1.y < a0.y,
                _ => true,
            };
            if Some(k) == except || apart {
                continue;
            }
            if rings_touch(points, &other.points)
                || polygon_contains(&other.points, points[0])
                || polygon_contains(points, other.points[0])
            {
                return Some(format!("overlaps cutout {}", k + 1));
            }
        }
        None
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
                || self.placements.iter().any(|p| p.bottom || p.has_back_copper_pad())
                || self.tracks.iter().any(|t| t.layer != 0))
        {
            return Err(
                "A single-layer board cannot keep vias, bottom tracks, bottom-side parts, or back-side pads".into(),
            );
        }
        // A via's span keeps its ends: an end on the bottom stays on the bottom,
        // an end on a removed inner layer refuses the change.
        let remap = |layer: u8| -> Option<u8> {
            if layer == 0 {
                Some(0)
            } else if layer == old_last {
                Some(new_last)
            } else if layer < new_last {
                Some(layer)
            } else {
                None
            }
        };
        let mut spans = vec![];
        for via in &self.vias {
            spans.push(match via.span {
                None => None,
                Some((from, to)) => match (remap(from), remap(to)) {
                    (Some(from), Some(to)) if from < to => Some((from, to)),
                    _ => return Err("Move or delete the blind and buried vias on removed inner layers first".into()),
                },
            });
        }
        // Both ends kept means it still stops short of one outer layer (or both):
        // a blind or buried via stays one.
        for (via, span) in self.vias.iter_mut().zip(spans) {
            via.span = span;
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
                layers: via.layers(self.layer_count),
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
                // Copper ON an edge (crossing the outline or a cutout) is a
                // finding even when the edge clearance is nil: the mill cuts it.
                // A cutout the fill wholly covers touches none of its edges: a
                // stale fill poured before the cutout was made.
                let covers_cutout = self.cutouts.iter().any(|c| piece.contains(c.points[0]));
                if !self.outline_contains(piece.outer[0])
                    || edge_gap + 0.5 < edge_clearance
                    || edge_gap == 0.
                    || covers_cutout
                {
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
            if (!self.outline_contains(center) || gap + 0.5 < edge_clearance || gap == 0.)
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
        // Courtyards side by side: a part's own on the side it is mounted on,
        // and a drawn back courtyard on the other.
        let overlap = |a: Option<(Point, Point)>, b: Option<(Point, Point)>| match (a, b) {
            (Some((a0, a1)), Some((b0, b1))) => a0.x < b1.x && b0.x < a1.x && a0.y < b1.y && b0.y < a1.y,
            _ => false,
        };
        for (i, a) in self.placements.iter().enumerate() {
            for b in &self.placements[i + 1..] {
                if [true, false].into_iter().any(|top| overlap(a.courtyard_on(top), b.courtyard_on(top))) {
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
        // A one-layer board has no bottom copper: a surface-mount pad that would
        // face the bottom (a back pad of a top part) has nowhere to go.
        if self.layer_count == 1 {
            for placement in self.placements.iter().filter(|p| p.has_back_copper_pad()) {
                violations.push(Violation {
                    kind: ViolationKind::MissingPad,
                    message: format!(
                        "{} has pads on the back copper, which a one-layer board does not have",
                        references.get(&placement.component).map_or("?", String::as_str)
                    ),
                    at: placement.at,
                });
            }
        }
        let ring = self.rules.min_annular_ring;
        for placement in &self.placements {
            for pad in &placement.footprint.pads {
                if let Some(have) = pad.annular_ring().filter(|r| *r < ring) {
                    let reference = references.get(&placement.component).map_or("?", String::as_str);
                    let name = if pad.number.is_empty() { "unnumbered pad".to_owned() } else { format!("pad {}", pad.number) };
                    violations.push(Violation {
                        kind: ViolationKind::AnnularRing,
                        message: format!(
                            "{reference} {name}: annular ring {:.3} mm, {:.3} mm required",
                            f64::from(have) / 1000.,
                            f64::from(ring) / 1000.
                        ),
                        at: placement.transform(pad.at),
                    });
                }
            }
        }
        for via in &self.vias {
            let have = (via.diameter - via.drill) / 2;
            if have < ring {
                violations.push(Violation {
                    kind: ViolationKind::AnnularRing,
                    message: format!(
                        "Via: annular ring {:.3} mm, {:.3} mm required",
                        f64::from(have) / 1000.,
                        f64::from(ring) / 1000.
                    ),
                    at: via.at,
                });
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
            || !(0..=10_000).contains(&r.min_annular_ring)
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
                    || pad.outline_problem().is_some()
                    || pad.layers.paste_windows.is_some_and(|w| w.problem().is_some())
                    || pad.hole_problem().is_some()
                    || pad.hole_size().is_some_and(|h| !size(h.x) || !size(h.y))
                    || !(0..=100_000).contains(&pad.corner_radius)
                    || [pad.layers.mask_margin, pad.layers.paste_margin]
                        .into_iter()
                        .flatten()
                        .any(|m| !(-10_000..=10_000).contains(&m))
                {
                    return Err(format!("Invalid pad in footprint {}", p.footprint.name));
                }
            }
            if p.footprint.courtyard_margin.is_some_and(|m| !(0..=100_000).contains(&m)) {
                return Err(format!("Invalid courtyard clearance in footprint {}", p.footprint.name));
            }
            if p.footprint.silk.iter().flatten().any(|q| !valid(*q)) {
                return Err("Invalid footprint silkscreen".into());
            }
            if p.footprint.graphics.len() > 100_000
                || p.footprint.graphics.iter().any(|g| g.problem().is_some())
            {
                return Err(format!("Invalid drawing in footprint {}", p.footprint.name));
            }
        }
        if self.graphics.len() > 100_000 {
            return Err("A board holds at most 100000 drawn items".into());
        }
        // A drawn item is selected, edited and deleted by its id: two sharing
        // one would be edited as one and deleted together.
        let mut ids = BTreeSet::new();
        if !self.graphics.iter().all(|g| ids.insert(g.id)) {
            return Err("Two board drawings share an id".into());
        }
        if let Some((n, problem)) = self
            .graphics
            .iter()
            .enumerate()
            .find_map(|(n, g)| g.problem().map(|p| (n, p)))
        {
            return Err(format!("Board drawing {} {problem}", n + 1));
        }
        if self.cutouts.len() > MAX_CUTOUTS {
            return Err(format!("A board holds at most {MAX_CUTOUTS} cutouts"));
        }
        // A cutout is selected, moved and deleted by its id, as a drawing is.
        let mut ids = BTreeSet::new();
        if !self.cutouts.iter().all(|c| ids.insert(c.id)) {
            return Err("Two board cutouts share an id".into());
        }
        for (n, c) in self.cutouts.iter().enumerate() {
            if c.points.iter().any(|q| !valid(*q)) {
                return Err(format!("Cutout {} has a corner off the drawing area", n + 1));
            }
            // Each against the ones before it, so of two that overlap the later
            // one, the one just drawn, is named.
            if let Some(problem) = self.cutout_problem_among(&c.points, &self.cutouts[..n], None) {
                return Err(format!("Cutout {} {problem}", n + 1));
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
            if v.span.is_some_and(|(from, to)| from >= to || to >= self.layer_count) {
                return Err("A blind or buried via needs two different layers of the board".into());
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
    /// middle of the board, then each part, in `Self::placement_order`, in the free
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
            // The seated parts' courtyards on the side this part is mounted on,
            // and on the other side, which a back courtyard of its own occupies.
            let mounted_top = !placement.bottom;
            let on_side = |top: bool| -> Vec<Shape> {
                seated
                    .iter()
                    .filter_map(|p| p.courtyard_on(top))
                    .map(|(min, max)| Shape::Rect { min, max })
                    .collect()
            };
            let (courtyards, other_side) = (on_side(mounted_top), on_side(!mounted_top));
            let back = placement.back_courtyard();
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
                    && back.is_none_or(|(b0, b1)| {
                        let back = Shape::Rect { min: b0.offset(at), max: b1.offset(at) };
                        other_side.iter().all(|c| back.distance(c) >= gap)
                    })
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
            .chain(self.board.cutouts.iter().flat_map(|c| c.points.clone()))
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
