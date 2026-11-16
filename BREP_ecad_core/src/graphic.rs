//! Drawn geometry on the non-copper layers of a footprint: silkscreen,
//! fabrication drawing and courtyard, on either side.
//!
//! A footprint's older `silk` polylines are front silkscreen at
//! [`crate::fabrication::SILK_WIDTH`]; [`crate::board::Footprint::drawing`] reads
//! both kinds as one list, so the legend, the 2D views and the 3D board need one
//! path.
use crate::font;
use crate::Point;
use serde::{Deserialize, Serialize};

/// The layer a [`LayerGraphic`] is drawn on, in the footprint's own frame: a part
/// mounted on the bottom puts its front layers on the board's back.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub enum GraphicLayer {
    #[serde(rename = "F.SilkS")]
    FrontSilk,
    #[serde(rename = "B.SilkS")]
    BackSilk,
    #[serde(rename = "F.Fab")]
    FrontFab,
    #[serde(rename = "B.Fab")]
    BackFab,
    #[serde(rename = "F.CrtYd")]
    FrontCourtyard,
    #[serde(rename = "B.CrtYd")]
    BackCourtyard,
}
impl GraphicLayer {
    pub const ALL: [Self; 6] = [
        Self::FrontSilk,
        Self::BackSilk,
        Self::FrontFab,
        Self::BackFab,
        Self::FrontCourtyard,
        Self::BackCourtyard,
    ];
    /// KiCad's name, which is also the stored one.
    pub fn name(self) -> &'static str {
        match self {
            Self::FrontSilk => "F.SilkS",
            Self::BackSilk => "B.SilkS",
            Self::FrontFab => "F.Fab",
            Self::BackFab => "B.Fab",
            Self::FrontCourtyard => "F.CrtYd",
            Self::BackCourtyard => "B.CrtYd",
        }
    }
    pub fn from_name(name: &str) -> Option<Self> {
        Some(match name {
            "F.SilkS" | "F.Silkscreen" => Self::FrontSilk,
            "B.SilkS" | "B.Silkscreen" => Self::BackSilk,
            "F.Fab" => Self::FrontFab,
            "B.Fab" => Self::BackFab,
            "F.CrtYd" | "F.Courtyard" => Self::FrontCourtyard,
            "B.CrtYd" | "B.Courtyard" => Self::BackCourtyard,
            _ => return None,
        })
    }
    pub fn front(self) -> bool {
        matches!(self, Self::FrontSilk | Self::FrontFab | Self::FrontCourtyard)
    }
    /// The same layer on the other side.
    pub fn flipped(self) -> Self {
        match self {
            Self::FrontSilk => Self::BackSilk,
            Self::BackSilk => Self::FrontSilk,
            Self::FrontFab => Self::BackFab,
            Self::BackFab => Self::FrontFab,
            Self::FrontCourtyard => Self::BackCourtyard,
            Self::BackCourtyard => Self::FrontCourtyard,
        }
    }
    pub fn is_silk(self) -> bool {
        matches!(self, Self::FrontSilk | Self::BackSilk)
    }
    pub fn is_courtyard(self) -> bool {
        matches!(self, Self::FrontCourtyard | Self::BackCourtyard)
    }
    /// The line width a graphic takes when it names none: KiCad's defaults.
    pub fn default_width(self) -> i32 {
        match self {
            Self::FrontSilk | Self::BackSilk => crate::fabrication::SILK_WIDTH,
            Self::FrontFab | Self::BackFab => 100,
            Self::FrontCourtyard | Self::BackCourtyard => 50,
        }
    }
}

/// What a [`LayerGraphic`] draws, in footprint coordinates (micrometres, Y down).
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum GraphicShape {
    /// A polyline; a rectangle or polygon is one whose last point is its first.
    Polyline { points: Vec<Point> },
    /// The circular arc from `start` through `mid` to `end`.
    Arc { start: Point, mid: Point, end: Point },
    Circle { center: Point, radius: i32 },
    /// Stroke-font text, centred on `at` with its baseline there, turned
    /// `angle` whole degrees counter-clockwise as seen (KiCad's sense) about
    /// `at`. An unturned text writes no angle, so older files read unchanged.
    Text {
        at: Point,
        text: String,
        height: i32,
        #[serde(default, skip_serializing_if = "no_angle")]
        angle: i32,
    },
}

/// Whether a text angle is none, which a file leaves out.
pub fn no_angle(angle: &i32) -> bool {
    *angle == 0
}

/// Whole degrees brought into `0..360`.
pub fn normal_angle(degrees: i64) -> i32 {
    degrees.rem_euclid(360) as i32
}

/// `p` turned `degrees` counter-clockwise as seen (y down) about `about`. A
/// quarter turn is exact; any other angle rounds to the micrometre.
pub fn turn_about(p: Point, about: Point, degrees: i32) -> Point {
    let (dx, dy) = (i64::from(p.x) - i64::from(about.x), i64::from(p.y) - i64::from(about.y));
    let (x, y) = match normal_angle(i64::from(degrees)) {
        0 => (dx, dy),
        90 => (dy, -dx),
        180 => (-dx, -dy),
        270 => (-dy, dx),
        a => {
            let (sin, cos) = f64::from(a).to_radians().sin_cos();
            let (fx, fy) = (dx as f64, dy as f64);
            ((fx * cos + fy * sin).round() as i64, (fy * cos - fx * sin).round() as i64)
        }
    };
    let clamp = |v: i64| v.clamp(i64::from(i32::MIN), i64::from(i32::MAX)) as i32;
    Point::new(clamp(i64::from(about.x) + x), clamp(i64::from(about.y) + y))
}

/// Up the letters of text turned `degrees` (counter-clockwise as seen), as a
/// unit vector on the canvas (y down): straight up the page when unturned.
pub fn text_up(degrees: i32) -> (f64, f64) {
    let (sin, cos) = f64::from(degrees).to_radians().sin_cos();
    (-sin, -cos)
}

/// One drawn item on a non-copper layer.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct LayerGraphic {
    pub layer: GraphicLayer,
    /// The pen width; `None` takes the layer's [`GraphicLayer::default_width`].
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub width: Option<i32>,
    /// A closed polyline or a circle is filled, not only outlined.
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub filled: bool,
    #[serde(flatten)]
    pub shape: GraphicShape,
}

/// Largest coordinate a graphic may use, as for every other footprint point.
const LIMIT: i32 = 100_000_000;
/// Longest text a graphic may carry.
pub const MAX_TEXT: usize = 200;
/// The smallest and largest text height a graphic may have, in micrometres.
pub const TEXT_HEIGHTS: std::ops::RangeInclusive<i32> = 100..=100_000;
/// Facets of a full circle when it is stroked as a polyline.
const CIRCLE_FACETS: i32 = 64;

impl LayerGraphic {
    pub fn new(layer: GraphicLayer, shape: GraphicShape) -> Self {
        Self { layer, width: None, filled: false, shape }
    }
    pub fn width(&self) -> i32 {
        self.width.unwrap_or(self.layer.default_width())
    }
    /// Whether the shape encloses an area that `filled` can fill.
    pub fn closed(&self) -> bool {
        match &self.shape {
            GraphicShape::Polyline { points } => points.len() >= 4 && points.first() == points.last(),
            GraphicShape::Circle { .. } => true,
            _ => false,
        }
    }
    /// Why this graphic cannot be stored, checked before any geometry is built
    /// from it.
    pub fn problem(&self) -> Option<&'static str> {
        let inside = |p: &Point| p.x.abs_diff(0) <= LIMIT as u32 && p.y.abs_diff(0) <= LIMIT as u32;
        // A filled shape may have no outline pen (KiCad's `(width 0)`).
        if !(1..=10_000).contains(&self.width()) && !(self.filled && self.width() == 0) {
            return Some("needs a line width between 0.001 and 10 mm");
        }
        match &self.shape {
            GraphicShape::Polyline { points } => {
                if points.len() < 2 {
                    return Some("needs at least two points");
                }
                if points.len() > 100_000 {
                    return Some("has more than 100000 points");
                }
                if !points.iter().all(inside) {
                    return Some("is outside the drawing area");
                }
            }
            GraphicShape::Arc { start, mid, end } => {
                // The curve itself, not only the points it is given: three
                // nearly collinear points make a circle far larger than they are.
                if ![start, mid, end].into_iter().all(inside)
                    || !arc_points(*start, *mid, *end).iter().all(inside)
                {
                    return Some("is outside the drawing area");
                }
            }
            GraphicShape::Circle { center, radius } => {
                if !inside(center) {
                    return Some("is outside the drawing area");
                }
                if !(1..=LIMIT).contains(radius) {
                    return Some("needs a radius between 0.001 mm and 100 m");
                }
            }
            GraphicShape::Text { at, text, height, angle } => {
                if !inside(at) {
                    return Some("is outside the drawing area");
                }
                if text.trim().is_empty() || text.chars().count() > MAX_TEXT {
                    return Some("needs text of 1 to 200 characters");
                }
                if !TEXT_HEIGHTS.contains(height) {
                    return Some("needs a text height between 0.1 and 100 mm");
                }
                if !(0..360).contains(angle) {
                    return Some("needs a text angle from 0 to 359 degrees");
                }
            }
        }
        if self.filled && !self.closed() {
            return Some("is filled but not closed");
        }
        None
    }
    /// The centre lines the pen follows, in footprint coordinates. `mirrored`
    /// writes text right to left, for text read from the back of the board.
    pub fn strokes(&self, mirrored: bool) -> Vec<Vec<Point>> {
        match &self.shape {
            GraphicShape::Polyline { points } => vec![points.clone()],
            GraphicShape::Arc { start, mid, end } => vec![arc_points(*start, *mid, *end)],
            GraphicShape::Circle { center, radius } => {
                let r = f64::from(*radius);
                vec![(0..=CIRCLE_FACETS)
                    .map(|i| {
                        let t = std::f64::consts::TAU * f64::from(i) / f64::from(CIRCLE_FACETS);
                        Point::new(center.x + (r * t.cos()).round() as i32, center.y + (r * t.sin()).round() as i32)
                    })
                    .collect()]
            }
            GraphicShape::Text { at, text, height, angle } => {
                let mut strokes = font::text_strokes(text, *at, *height, mirrored, &mut Default::default());
                if *angle != 0 {
                    for p in strokes.iter_mut().flatten() {
                        *p = turn_about(*p, *at, *angle);
                    }
                }
                strokes
            }
        }
    }
    /// Every point the graphic's centre line reaches, for bounds.
    pub fn points(&self) -> Vec<Point> {
        self.strokes(false).into_iter().flatten().collect()
    }
}

/// The arc from `a` through `m` to `b` as 33 points; three collinear points are
/// the polyline through them.
pub fn arc_points(a: Point, m: Point, b: Point) -> Vec<Point> {
    let (ax, ay, mx, my, bx, by) = (
        a.x as f64, a.y as f64, m.x as f64, m.y as f64, b.x as f64, b.y as f64,
    );
    let d = 2. * (ax * (my - by) + mx * (by - ay) + bx * (ay - my));
    if d.abs() < 0.01 {
        return vec![a, m, b];
    }
    let (aa, mm, bb) = (ax * ax + ay * ay, mx * mx + my * my, bx * bx + by * by);
    let cx = (aa * (my - by) + mm * (by - ay) + bb * (ay - my)) / d;
    let cy = (aa * (bx - mx) + mm * (ax - bx) + bb * (mx - ax)) / d;
    let start = (ay - cy).atan2(ax - cx);
    let mid = (my - cy).atan2(mx - cx);
    let end = (by - cy).atan2(bx - cx);
    let tau = std::f64::consts::TAU;
    let ccw = (end - start).rem_euclid(tau);
    let sweep = if (mid - start).rem_euclid(tau) <= ccw {
        ccw
    } else {
        ccw - tau
    };
    let r = ((ax - cx).powi(2) + (ay - cy).powi(2)).sqrt();
    // A far arc saturates at the i32 range (`as` clamps), which is far outside
    // the drawing area, so [`LayerGraphic::problem`] refuses it.
    (0..=32)
        .map(|i| {
            let t = start + sweep * i as f64 / 32.;
            Point::new(
                (cx + r * t.cos()).round() as i32,
                (cy + r * t.sin()).round() as i32,
            )
        })
        .collect()
}
