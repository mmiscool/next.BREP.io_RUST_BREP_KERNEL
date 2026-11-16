//! Grab handles: the small squares a selected shape wears at its corners and
//! edges, its radius, its own corners, so it can be sized by dragging rather
//! than only by typing in the Inspector. Geometry only, in the editors'
//! micrometres with y down as on the canvas; the editors draw the handles, hit
//! them and record the drag.
//!
//! A drag always starts from the shape as it was when the handle was pressed
//! and is applied afresh each frame (the editors keep the "before" copy), so a
//! drag that goes back where it started leaves the shape exactly as it was.
use brep_ecad_core::Point;
use brep_ecad_core::board::{Pad, PadShape};
use brep_ecad_core::graphic::{GraphicShape, TEXT_HEIGHTS, normal_angle, text_up};
use egui::{Color32, CursorIcon, Pos2, Rect, Stroke, Vec2};

/// One handle of a shape.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub(crate) enum Handle {
    /// A corner or the middle of an edge of the shape's box: `x` and `y` are
    /// -1 (left/top), 0 (middle) or 1 (right/bottom), never both 0. Dragging it
    /// moves that side or corner; the opposite one stays put.
    Box(i8, i8),
    /// A circle's radius: its centre stays put.
    Radius,
    /// A polygon's or a line's own corner, by index.
    Vertex(usize),
    /// A rounded rectangle's corner radius, on its top edge where the rounding
    /// ends.
    Rounding,
    /// A trapezoid's taper, on its top-right corner: dragged up or down it
    /// tapers the left and right sides, left or right the top and bottom.
    Taper,
    /// Text's height, on the side of its letters away from their anchor (in
    /// Pads and on the board the top of the letters over the baseline, in
    /// Symbol their foot below the top-left corner): the anchor holds and the
    /// letters grow or shrink as it is dragged.
    Height,
}

/// The smallest side a drag leaves a shape with, in micrometres: a box dragged
/// past its opposite side stops there instead of turning inside out.
pub(crate) const LEAST: i32 = 10;

impl Handle {
    /// The name automation and the status strip know the handle by, after the
    /// item's own key: `ne`, `e`, `radius`, `vertex:3`, `rounding`, `taper`.
    pub(crate) fn name(self) -> String {
        match self {
            Handle::Box(x, y) => {
                let v = match y {
                    -1 => "n",
                    1 => "s",
                    _ => "",
                };
                let h = match x {
                    -1 => "w",
                    1 => "e",
                    _ => "",
                };
                format!("{v}{h}")
            }
            Handle::Radius => "radius".into(),
            Handle::Vertex(i) => format!("vertex:{i}"),
            Handle::Rounding => "rounding".into(),
            Handle::Taper => "taper".into(),
            Handle::Height => "height".into(),
        }
    }
    /// Where the handle is drawn and taken, from the screen point of its place:
    /// a rounding's diamond sits a little inside the pad, below the top edge,
    /// so with no rounding at all it is not under the corner's handle. Its drag
    /// reads only across the edge, so the offset changes nothing it sets.
    pub(crate) fn on_screen(self, at: Pos2) -> Pos2 {
        match self {
            Handle::Rounding => at + Vec2::new(0., 2. * REACH),
            _ => at,
        }
    }
    /// The pointer over the handle says which way it drags.
    pub(crate) fn cursor(self) -> CursorIcon {
        match self {
            Handle::Box(-1, -1) | Handle::Box(1, 1) => CursorIcon::ResizeNwSe,
            Handle::Box(1, -1) | Handle::Box(-1, 1) => CursorIcon::ResizeNeSw,
            Handle::Box(_, 0) => CursorIcon::ResizeHorizontal,
            Handle::Box(0, _) | Handle::Height => CursorIcon::ResizeVertical,
            Handle::Box(..) | Handle::Radius | Handle::Rounding | Handle::Taper => CursorIcon::Crosshair,
            Handle::Vertex(_) => CursorIcon::Move,
        }
    }
}

/// The eight handles of the box `min..max`, corners first.
pub(crate) fn box_handles(min: Point, max: Point) -> Vec<(Handle, Point)> {
    let x = |s: i8| match s {
        -1 => min.x,
        1 => max.x,
        _ => mid(min.x, max.x),
    };
    let y = |s: i8| match s {
        -1 => min.y,
        1 => max.y,
        _ => mid(min.y, max.y),
    };
    [(-1, -1), (1, -1), (1, 1), (-1, 1), (0, -1), (1, 0), (0, 1), (-1, 0)]
        .into_iter()
        .map(|(sx, sy)| (Handle::Box(sx, sy), Point::new(x(sx), y(sy))))
        .collect()
}

fn mid(a: i32, b: i32) -> i32 {
    (i64::from(a) + i64::from(b)).div_euclid(2) as i32
}

fn clamp32(v: i64) -> i32 {
    v.clamp(-100_000_000, 100_000_000) as i32
}

/// The box `min..max` with its `(sx, sy)` corner or edge dragged to `q`, the
/// opposite side held. `keep` (Shift) keeps the box's proportions: a corner
/// scales both sides by the larger stretch, an edge scales the other side
/// about its middle.
pub(crate) fn resize_box(min: Point, max: Point, (sx, sy): (i8, i8), q: Point, keep: bool) -> (Point, Point) {
    let (x0, x1, y0, y1) = (i64::from(min.x), i64::from(max.x), i64::from(min.y), i64::from(max.y));
    let least = i64::from(LEAST);
    // One axis: the side `s` moves to `to`, the other stays.
    let axis = |s: i8, a: i64, b: i64, to: i64| -> (i64, i64) {
        match s {
            1 => (a, to.max(a + least)),
            -1 => (to.min(b - least), b),
            _ => (a, b),
        }
    };
    let (mut nx0, mut nx1) = axis(sx, x0, x1, i64::from(q.x));
    let (mut ny0, mut ny1) = axis(sy, y0, y1, i64::from(q.y));
    let (w0, h0) = ((x1 - x0).max(1) as f64, (y1 - y0).max(1) as f64);
    if keep {
        let (fx, fy) = ((nx1 - nx0) as f64 / w0, (ny1 - ny0) as f64 / h0);
        let f = match (sx, sy) {
            (0, _) => fy,
            (_, 0) => fx,
            _ => fx.max(fy),
        };
        let w = (w0 * f).round().max(least as f64) as i64;
        let h = (h0 * f).round().max(least as f64) as i64;
        // Each side grows away from what holds it: the opposite side, or for
        // an edge's other axis, its middle.
        let place = |s: i8, a: i64, b: i64, len: i64| -> (i64, i64) {
            match s {
                1 => (a, a + len),
                -1 => (b - len, b),
                _ => {
                    let c = a + b;
                    ((c - len) / 2, (c - len) / 2 + len)
                }
            }
        };
        (nx0, nx1) = place(sx, x0, x1, w);
        (ny0, ny1) = place(sy, y0, y1, h);
    }
    (Point::new(clamp32(nx0), clamp32(ny0)), Point::new(clamp32(nx1), clamp32(ny1)))
}

/// A pad's box: its position, less and plus half its size.
fn pad_box(pad: &Pad) -> (Point, Point) {
    let (w, h) = (pad.size.x.abs(), pad.size.y.abs());
    let min = Point::new(pad.at.x - w / 2, pad.at.y - h / 2);
    (min, Point::new(min.x + w, min.y + h))
}

/// The handles a selected pad wears: a circle its radius; a custom pad its own
/// corners; every other shape its box, a rounded rectangle its rounding too and
/// a trapezoid its taper.
pub(crate) fn pad_handles(pad: &Pad) -> Vec<(Handle, Point)> {
    let (min, max) = pad_box(pad);
    match pad.shape {
        PadShape::Circle => {
            let r = pad.size.x.abs().min(pad.size.y.abs()) / 2;
            vec![(Handle::Radius, Point::new(pad.at.x + r, pad.at.y))]
        }
        PadShape::Custom => pad
            .polygon
            .iter()
            .enumerate()
            .map(|(i, p)| (Handle::Vertex(i), pad.at.offset(*p)))
            .collect(),
        PadShape::Trapezoid => {
            // The box's edges, not its corners: the shape's corners are the
            // taper's, and the taper's handle sits on the top-right one.
            let mut handles: Vec<_> = box_handles(min, max).into_iter().filter(|(h, _)| matches!(h, Handle::Box(0, _) | Handle::Box(_, 0))).collect();
            if let Some(corners) = pad.corners() {
                handles.push((Handle::Taper, corners[2]));
            }
            handles
        }
        PadShape::RoundRect => {
            let mut handles = box_handles(min, max);
            handles.push((Handle::Rounding, Point::new(min.x + pad.effective_corner_radius(), min.y)));
            handles
        }
        PadShape::Rect | PadShape::Oval => box_handles(min, max),
    }
}

/// `before` with its `handle` dragged to `q`, into `pad`.
pub(crate) fn drag_pad(pad: &mut Pad, before: &Pad, handle: Handle, q: Point, keep: bool) {
    *pad = before.clone();
    let (min, max) = pad_box(before);
    match handle {
        Handle::Box(sx, sy) => {
            let (min, max) = resize_box(min, max, (sx, sy), q, keep);
            pad.size = Point::new(max.x - min.x, max.y - min.y);
            // The position a pad's own box is built from (less half its size,
            // rounded as `pad_box` rounds it), so both edges land exactly.
            pad.at = Point::new(min.x + pad.size.x / 2, min.y + pad.size.y / 2);
            // A trapezoid's taper stays shorter than the side it narrows.
            let d = before.trapezoid_delta;
            pad.trapezoid_delta =
                Point::new(d.x.clamp(-(pad.size.y - 1), pad.size.y - 1), d.y.clamp(-(pad.size.x - 1), pad.size.x - 1));
        }
        Handle::Radius => {
            let (dx, dy) = (f64::from(q.x) - f64::from(before.at.x), f64::from(q.y) - f64::from(before.at.y));
            let d = clamp32((2. * dx.hypot(dy)).round() as i64).max(LEAST);
            pad.size = Point::new(d, d);
        }
        Handle::Vertex(i) => {
            if let Some(p) = pad.polygon.get_mut(i) {
                *p = Point::new(clamp32(i64::from(q.x) - i64::from(before.at.x)), clamp32(i64::from(q.y) - i64::from(before.at.y)));
            }
        }
        Handle::Rounding => {
            let most = (max.x - min.x).min(max.y - min.y) / 2;
            pad.corner_radius = (q.x - min.x).clamp(0, most);
        }
        Handle::Taper => {
            // Relative to the untapered top-right corner (hx, -hy): moved more
            // up or down tapers the sides (`x`), more left or right the top and
            // bottom (`y`). One pair only, as a trapezoid pad allows.
            let (hx, hy) = (i64::from(before.size.x.abs()) / 2, i64::from(before.size.y.abs()) / 2);
            let (rx, ry) = (i64::from(q.x) - i64::from(before.at.x), i64::from(q.y) - i64::from(before.at.y));
            let (along_x, along_y) = (2 * (ry + hy), 2 * (hx - rx));
            pad.trapezoid_delta = if along_x.abs() >= along_y.abs() {
                let most = i64::from(before.size.y.abs()) - 1;
                Point::new(clamp32(along_x.clamp(-most, most)), 0)
            } else {
                let most = i64::from(before.size.x.abs()) - 1;
                Point::new(0, clamp32(along_y.clamp(-most, most)))
            };
        }
        // A pad has no text.
        Handle::Height => {}
    }
}

/// Whether a polyline is an upright rectangle: a closed ring of four corners
/// whose edges run along the axes. Such a one is sized by its box; any other
/// by its own corners.
pub(crate) fn upright_rectangle(points: &[Point]) -> Option<(Point, Point)> {
    if points.len() != 5 || points[0] != points[4] {
        return None;
    }
    // Each edge along one axis, turning at every corner (so a line traced out
    // and back along itself is not a rectangle) ...
    let along_x = |a: Point, b: Point| a.y == b.y && a.x != b.x;
    let along_y = |a: Point, b: Point| a.x == b.x && a.y != b.y;
    let first_x = along_x(points[0], points[1]);
    let upright = points.windows(2).enumerate().all(|(k, s)| {
        if (k % 2 == 0) == first_x { along_x(s[0], s[1]) } else { along_y(s[0], s[1]) }
    });
    if !upright {
        return None;
    }
    // ... and its four corners the four corners of its box.
    let min = Point::new(points.iter().map(|p| p.x).min()?, points.iter().map(|p| p.y).min()?);
    let max = Point::new(points.iter().map(|p| p.x).max()?, points.iter().map(|p| p.y).max()?);
    let corner = |p: &Point| (p.x == min.x || p.x == max.x) && (p.y == min.y || p.y == max.y);
    let distinct = (0..4).all(|i| (0..i).all(|j| points[i] != points[j]));
    (distinct && points[..4].iter().all(corner)).then_some((min, max))
}

/// The corners of a polyline a drag can move: every point, but a closed ring's
/// last point is its first and has no handle of its own.
fn polyline_vertices(points: &[Point]) -> Vec<(Handle, Point)> {
    let closed = points.len() > 2 && points.first() == points.last();
    let n = if closed { points.len() - 1 } else { points.len() };
    points[..n].iter().enumerate().map(|(i, p)| (Handle::Vertex(i), *p)).collect()
}

/// The polyline `before` with its corner `i` dragged to `q`, a closed ring's
/// first and last point moving together.
fn drag_vertex(points: &mut [Point], i: usize, q: Point) {
    let closed = points.len() > 2 && points.first() == points.last();
    if let Some(p) = points.get_mut(i) {
        *p = q;
    }
    if closed && i == 0 {
        let last = points.len() - 1;
        points[last] = q;
    }
}

/// The handles of a polyline: an upright rectangle's box, else its corners.
pub(crate) fn path_handles(points: &[Point]) -> Vec<(Handle, Point)> {
    match upright_rectangle(points) {
        Some((min, max)) => box_handles(min, max),
        None => polyline_vertices(points),
    }
}

/// The polyline `before` with its `handle` dragged to `q`, into `points`.
pub(crate) fn drag_path(points: &mut Vec<Point>, before: &[Point], handle: Handle, q: Point, keep: bool) {
    *points = before.to_vec();
    match handle {
        Handle::Box(sx, sy) => {
            if let Some((min, max)) = upright_rectangle(before) {
                let (nmin, nmax) = resize_box(min, max, (sx, sy), q, keep);
                for p in points.iter_mut() {
                    p.x = if p.x == min.x { nmin.x } else { nmax.x };
                    p.y = if p.y == min.y { nmin.y } else { nmax.y };
                }
            }
        }
        Handle::Vertex(i) => drag_vertex(points, i, q),
        _ => {}
    }
}

/// A circle's radius dragged to `q`, its centre held.
pub(crate) fn radius_to(center: Point, q: Point) -> i32 {
    let (dx, dy) = (f64::from(q.x) - f64::from(center.x), f64::from(q.y) - f64::from(center.y));
    clamp32(dx.hypot(dy).round() as i64).max(LEAST)
}

/// Where a text's height handle sits: `height` up from its anchor `at`, along
/// `up` (a unit vector).
pub(crate) fn text_height_point(at: Point, up: (f64, f64), height: i32) -> Point {
    let h = f64::from(height);
    Point::new(clamp32((f64::from(at.x) + up.0 * h).round() as i64), clamp32((f64::from(at.y) + up.1 * h).round() as i64))
}

/// A text's height with its handle dragged to `q`: how far `q` is from the
/// anchor `at` along `up`, held within `least..=most`. Only the reach along
/// `up` counts, so a pointer that wanders sideways sizes the text the same.
pub(crate) fn text_height_to(at: Point, up: (f64, f64), q: Point, least: i32, most: i32) -> i32 {
    let (dx, dy) = (f64::from(q.x) - f64::from(at.x), f64::from(q.y) - f64::from(at.y));
    clamp32((dx * up.0 + dy * up.1).round() as i64).clamp(least, most)
}

/// The handles a drawn item wears: an upright rectangle its box, any other line
/// or polygon its corners, a circle its radius, an arc its three points, text
/// its height.
pub(crate) fn graphic_handles(shape: &GraphicShape) -> Vec<(Handle, Point)> {
    match shape {
        GraphicShape::Polyline { points } => path_handles(points),
        GraphicShape::Circle { center, radius } => vec![(Handle::Radius, Point::new(center.x + radius, center.y))],
        GraphicShape::Arc { start, mid, end } => {
            vec![(Handle::Vertex(0), *start), (Handle::Vertex(1), *mid), (Handle::Vertex(2), *end)]
        }
        GraphicShape::Text { at, height, angle, .. } => {
            vec![(Handle::Height, text_height_point(*at, text_up(*angle), *height))]
        }
    }
}

/// `before` with its `handle` dragged to `q`, into `shape`.
pub(crate) fn drag_graphic(shape: &mut GraphicShape, before: &GraphicShape, handle: Handle, q: Point, keep: bool) {
    *shape = before.clone();
    match (shape, handle) {
        (GraphicShape::Polyline { points }, _) => {
            let was = points.clone();
            drag_path(points, &was, handle, q, keep);
        }
        (GraphicShape::Circle { center, radius }, Handle::Radius) => *radius = radius_to(*center, q),
        (GraphicShape::Arc { start, mid, end }, Handle::Vertex(i)) => {
            if let Some(p) = [start, mid, end].into_iter().nth(i) {
                *p = q;
            }
        }
        // Held within the heights a drawing may have, so a drag past either
        // stops there rather than being refused back to where it began.
        (GraphicShape::Text { at, height, angle, .. }, Handle::Height) => {
            let (least, most) = (*TEXT_HEIGHTS.start(), *TEXT_HEIGHTS.end());
            *height = text_height_to(*at, text_up(*angle), q, least, most);
        }
        _ => {}
    }
}

/// The handles of a plain ring of corners (a board cutout): its box when it is
/// an upright rectangle, else each corner.
pub(crate) fn ring_handles(points: &[Point]) -> Vec<(Handle, Point)> {
    let mut closed = points.to_vec();
    if let Some(first) = points.first() {
        closed.push(*first);
    }
    match upright_rectangle(&closed) {
        Some((min, max)) => box_handles(min, max),
        None => points.iter().enumerate().map(|(i, p)| (Handle::Vertex(i), *p)).collect(),
    }
}

/// `before`, a ring of corners, with its `handle` dragged to `q`.
pub(crate) fn drag_ring(points: &mut Vec<Point>, before: &[Point], handle: Handle, q: Point, keep: bool) {
    *points = before.to_vec();
    match handle {
        Handle::Box(sx, sy) => {
            let mut closed = before.to_vec();
            closed.push(before[0]);
            if let Some((min, max)) = upright_rectangle(&closed) {
                let (nmin, nmax) = resize_box(min, max, (sx, sy), q, keep);
                for p in points.iter_mut() {
                    p.x = if p.x == min.x { nmin.x } else { nmax.x };
                    p.y = if p.y == min.y { nmin.y } else { nmax.y };
                }
            }
        }
        Handle::Vertex(i) => {
            if let Some(p) = points.get_mut(i) {
                *p = q;
            }
        }
        _ => {}
    }
}

/// The largest coordinate any drawn point may have, as the drawings allow.
const FAR: i64 = 100_000_000;

/// The points turned a quarter turn, (x, y) to (−y, x) as a pad turns, about
/// `about`. A turn that would carry a point past the drawing area is not made
/// (`false`, the points untouched) rather than squeezed onto its edge.
pub(crate) fn turn_points(points: &mut [Point], about: Point) -> bool {
    let turned: Vec<(i64, i64)> = points
        .iter()
        .map(|p| {
            let (x, y) = (i64::from(p.x) - i64::from(about.x), i64::from(p.y) - i64::from(about.y));
            (i64::from(about.x) - y, i64::from(about.y) + x)
        })
        .collect();
    if turned.iter().any(|&(x, y)| x.abs() > FAR || y.abs() > FAR) {
        return false;
    }
    for (p, (x, y)) in points.iter_mut().zip(turned) {
        *p = Point::new(x as i32, y as i32);
    }
    true
}

/// The point a shape turns about so that a turn is truly in place: the middle
/// of its box when that is a whole micrometre, which a turn leaves where it
/// is (so four turns put the shape back exactly); else its first point, which
/// a turn about it leaves where it is too.
pub(crate) fn pivot(points: &[Point]) -> Option<Point> {
    let lo = Point::new(points.iter().map(|p| p.x).min()?, points.iter().map(|p| p.y).min()?);
    let hi = Point::new(points.iter().map(|p| p.x).max()?, points.iter().map(|p| p.y).max()?);
    let (sx, sy) = (i64::from(lo.x) + i64::from(hi.x), i64::from(lo.y) + i64::from(hi.y));
    if sx % 2 == 0 && sy % 2 == 0 {
        Some(Point::new((sx / 2) as i32, (sy / 2) as i32))
    } else {
        points.first().copied()
    }
}

/// A drawn shape turned a quarter turn about `about`, as a pad turns: text
/// turns its letters with it. A circle looks the same turned, and a turn past
/// the drawing area is not made: those answer `false`, the shape untouched.
pub(crate) fn turn_graphic(shape: &mut GraphicShape, about: Point) -> bool {
    match shape {
        GraphicShape::Polyline { points } => turn_points(points, about),
        GraphicShape::Arc { start, mid, end } => {
            let mut three = [*start, *mid, *end];
            let turned = turn_points(&mut three, about);
            [*start, *mid, *end] = three;
            turned
        }
        GraphicShape::Text { at, angle, .. } => {
            let mut one = [*at];
            let turned = turn_points(&mut one, about);
            if turned {
                [*at] = one;
                *angle = turned_angle(*angle);
            }
            turned
        }
        GraphicShape::Circle { .. } => false,
    }
}

/// A text angle after the quarter turn R makes, (x, y) to (−y, x) on the
/// canvas: clockwise as seen, so a quarter less counter-clockwise.
pub(crate) fn turned_angle(angle: i32) -> i32 {
    normal_angle(i64::from(angle) - 90)
}

/// The points a drawn shape is turned about: its corners or its arc's three.
pub(crate) fn graphic_points(shape: &GraphicShape) -> Vec<Point> {
    match shape {
        GraphicShape::Polyline { points } => points.clone(),
        GraphicShape::Arc { start, mid, end } => vec![*start, *mid, *end],
        GraphicShape::Circle { center, .. } => vec![*center],
        GraphicShape::Text { at, .. } => vec![*at],
    }
}

/// How far from a handle's centre a press still takes it, in points.
pub(crate) const REACH: f32 = 6.;

/// The handle of `handles` under the screen point `p`, the nearest if several.
pub(crate) fn handle_at(handles: &[(Handle, Point)], screen: impl Fn(Point) -> Pos2, p: Pos2) -> Option<Handle> {
    handles
        .iter()
        .map(|(h, at)| (*h, h.on_screen(screen(*at)).distance(p)))
        .filter(|(_, d)| *d <= REACH)
        .min_by(|a, b| a.1.total_cmp(&b.1))
        .map(|(h, _)| h)
}

/// A handle's rect on screen, as drawn and as published to automation, from
/// the screen point of its place.
pub(crate) fn handle_rect(handle: Handle, at: Pos2) -> Rect {
    Rect::from_center_size(handle.on_screen(at), Vec2::splat(2. * REACH))
}

/// Draw the handles: white squares with a dark edge (a circle for a radius, a
/// diamond for a rounding or a taper), the one under the pointer filled with
/// `accent`.
pub(crate) fn paint_handles(
    painter: &egui::Painter,
    handles: &[(Handle, Point)],
    screen: impl Fn(Point) -> Pos2,
    hot: Option<Handle>,
    accent: Color32,
) {
    let edge = Stroke::new(1., Color32::from_gray(20));
    for (h, at) in handles {
        let c = h.on_screen(screen(*at));
        let fill = if hot == Some(*h) { accent } else { Color32::WHITE };
        match h {
            Handle::Radius => {
                painter.circle(c, 4.5, fill, edge);
            }
            Handle::Rounding | Handle::Taper => {
                let r = 5.;
                let ring = vec![c + Vec2::new(0., -r), c + Vec2::new(r, 0.), c + Vec2::new(0., r), c + Vec2::new(-r, 0.)];
                painter.add(egui::Shape::convex_polygon(ring, fill, edge));
            }
            Handle::Box(..) | Handle::Vertex(_) | Handle::Height => {
                painter.rect(Rect::from_center_size(c, Vec2::splat(7.)), 0., fill, edge, egui::StrokeKind::Middle);
            }
        }
    }
}

/// What a held handle sets, written beside the pointer on a dark plate so it
/// reads over pads and drawing alike.
pub(crate) fn paint_readout(painter: &egui::Painter, pointer: Pos2, text: String) {
    let galley = painter.layout_no_wrap(text, egui::FontId::proportional(13.), Color32::WHITE);
    let at = pointer + Vec2::new(16., -16. - galley.size().y);
    let plate = Rect::from_min_size(at, galley.size()).expand(3.);
    painter.rect_filled(plate, 3., Color32::from_black_alpha(200));
    painter.galley(at, galley, Color32::WHITE);
}

/// Millimetres for a live readout: up to three places, trailing zeros dropped.
pub(crate) fn mm(v: i32) -> String {
    let s = format!("{:.3}", f64::from(v) / 1000.);
    s.trim_end_matches('0').trim_end_matches('.').to_owned()
}

