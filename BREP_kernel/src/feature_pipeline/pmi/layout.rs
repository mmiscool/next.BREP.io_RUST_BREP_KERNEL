//! PMI presentation layout — the polylines an annotation is DRAWN with, laid
//! out from its resolved geometry and its label position. ONE implementation
//! serves the viewport overlay (screen-constant sizes, from the camera's
//! world-per-pixel) and the AP242 polyline presentation (fixed model-unit
//! sizes from the view's text size), so the file shows what the screen shows.

use super::{font, PmiGeometry, PmiPlane};
use crate::feature_pipeline::pmi::annotations::angle::rotate;
use crate::feature_pipeline::pmi::resolve::{a3, perpendicular_in_plane, v3};
use crate::Vec3;

/// Sizing and orientation inputs.
#[derive(Debug, Clone, Copy)]
pub struct LayoutStyle {
    /// Arrowhead length (world units).
    pub arrow: f64,
    /// Text cap height (world units) — sizes datum / FCF frames.
    pub text_height: f64,
    /// The unit viewing direction (eye → target): arrowheads and frames lie
    /// in the plane perpendicular to it.
    pub view_dir: [f64; 3],
    /// The camera's unit up vector.
    pub view_up: [f64; 3],
    /// The annotation's picked plane: the drawing lies IN it (label and
    /// measured points projected onto it, its normal and text direction
    /// replacing the camera's). `None` aligns to the camera.
    pub plane: Option<PmiPlane>,
}

/// The drawn presentation.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Presentation {
    /// Open polylines: dimension lines, extension lines, leaders, arcs, trace
    /// segments.
    pub polylines: Vec<Vec<[f64; 3]>>,
    /// Filled arrowheads (and the datum triangle), as triangles.
    pub arrows: Vec<[[f64; 3]; 3]>,
    /// Closed frames (datum box, FCF cells) — drawn in the STEP presentation;
    /// the viewport draws its label chip instead.
    pub frames: Vec<Vec<[f64; 3]>>,
    /// The text and its anchor (the label position) — the viewport's chip.
    pub text: String,
    pub text_anchor: [f64; 3],
    /// The text as placed runs — one centred on the label, or the datum
    /// letter in its box, or one per FCF cell — which the STEP presentation
    /// strokes into polylines ([`TextRun::strokes`]).
    pub texts: Vec<TextRun>,
}

/// A run of text centred on `anchor`, read along `right` with `up`, at cap
/// height `height`.
#[derive(Debug, Clone, PartialEq)]
pub struct TextRun {
    pub text: String,
    pub anchor: [f64; 3],
    pub right: [f64; 3],
    pub up: [f64; 3],
    pub height: f64,
}

impl TextRun {
    /// The run stroked with the PMI font, centred on its anchor.
    pub fn strokes(&self) -> Vec<Vec<[f64; 3]>> {
        let width = font::text_width(&self.text, self.height);
        let origin = v3(self.anchor)
            .sub(v3(self.right).scale(width * 0.5))
            .sub(v3(self.up).scale(self.height * 0.5));
        font::stroke_text(&self.text, a3(origin), self.right, self.up, self.height)
    }
}

fn unit_or(v: Vec3, fallback: Vec3) -> Vec3 {
    if v.length() < 1e-12 {
        fallback
    } else {
        v.normalized().unwrap_or(fallback)
    }
}

/// The in-view width direction of an arrow along `dir`.
fn arrow_width_dir(dir: Vec3, style: &LayoutStyle) -> Vec3 {
    let view = v3(style.view_dir);
    let w = view.cross(dir);
    if w.length() < 1e-6 {
        dir.perpendicular().unwrap_or(Vec3::new(0.0, 1.0, 0.0))
    } else {
        w.normalized().unwrap_or(Vec3::new(0.0, 1.0, 0.0))
    }
}

/// An arrowhead with its tip at `tip`, pointing along `dir` (the body
/// trails behind the tip).
fn arrowhead(tip: Vec3, dir: Vec3, style: &LayoutStyle) -> [[f64; 3]; 3] {
    let dir = unit_or(dir, Vec3::new(1.0, 0.0, 0.0));
    let w = arrow_width_dir(dir, style);
    let base = tip.sub(dir.scale(style.arrow));
    let half = style.arrow * 0.35;
    [a3(tip), a3(base.add(w.scale(half))), a3(base.sub(w.scale(half)))]
}

/// A small closed square around `center` in the view plane (the leader's
/// dot end style).
fn dot(center: Vec3, style: &LayoutStyle) -> Vec<[f64; 3]> {
    let view = v3(style.view_dir);
    let up = perpendicular_in_plane(view, v3(style.view_up));
    let right = view.cross(up);
    let r = style.arrow * 0.25;
    let mut out = Vec::with_capacity(9);
    for k in 0..=8 {
        let angle = k as f64 * std::f64::consts::TAU / 8.0;
        out.push(a3(center.add(right.scale(r * angle.cos())).add(up.scale(r * angle.sin()))));
    }
    out
}

/// A rectangle of `width` × `height` whose left edge is centred at `left` in
/// the view plane.
fn frame_rect(left: Vec3, width: f64, height: f64, right: Vec3, up: Vec3) -> Vec<[f64; 3]> {
    let h = up.scale(height * 0.5);
    let w = right.scale(width);
    vec![
        a3(left.sub(h)),
        a3(left.add(w).sub(h)),
        a3(left.add(w).add(h)),
        a3(left.add(h)),
        a3(left.sub(h)),
    ]
}

/// Lay out `geometry` with its label at `label`.
pub fn present(geometry: &PmiGeometry, label: [f64; 3], text: &str, style: &LayoutStyle) -> Presentation {
    let mut out = Presentation {
        text: text.to_string(),
        text_anchor: label,
        ..Default::default()
    };
    // In a picked annotation plane the drawing lies in the plane: the
    // plane's normal (facing the camera) and text direction stand in for
    // the camera's, and the label / measured points project onto it.
    let plane = style.plane;
    let project = |p: Vec3| -> Vec3 {
        match &plane {
            Some(plane) => v3(plane.project(a3(p))),
            None => p,
        }
    };
    let in_plane;
    let style = match &plane {
        Some(plane) => {
            let n = v3(plane.normal);
            in_plane = LayoutStyle {
                view_dir: a3(n.scale(-1.0)),
                view_up: plane.y_axis(),
                ..*style
            };
            &in_plane
        }
        None => style,
    };
    let label = project(v3(label));
    out.text_anchor = a3(label);
    let view = v3(style.view_dir);
    let up = perpendicular_in_plane(view, v3(style.view_up));
    let right = view.cross(up);
    match geometry {
        PmiGeometry::None | PmiGeometry::Note { .. } => {}
        PmiGeometry::Linear { a, b, .. } => {
            let a = project(v3(*a));
            let b = project(v3(*b));
            let span = b.sub(a);
            let length = span.length();
            if length < 1e-9 {
                out.polylines.push(vec![a3(label), a3(a)]);
                return out;
            }
            let d = span.scale(1.0 / length);
            let rel = label.sub(a);
            let t = rel.dot(d);
            let off = rel.sub(d.scale(t));
            let a_line = a.add(off);
            let b_line = b.add(off);
            // The dimension line always reaches the label's station.
            let lo = t.min(0.0);
            let hi = t.max(length);
            out.polylines.push(vec![a3(a.add(off).add(d.scale(lo))), a3(a.add(off).add(d.scale(hi)))]);
            if off.length() > 1e-9 {
                let n = off.normalized().unwrap_or(Vec3::new(0.0, 1.0, 0.0));
                let overshoot = n.scale(style.arrow * 0.6);
                out.polylines.push(vec![a3(a), a3(a_line.add(overshoot))]);
                out.polylines.push(vec![a3(b), a3(b_line.add(overshoot))]);
            }
            if length > style.arrow * 2.5 {
                out.arrows.push(arrowhead(a_line, d.scale(-1.0), style));
                out.arrows.push(arrowhead(b_line, d, style));
            } else {
                // Too short for inside arrows: outside, pointing at the ends.
                out.arrows.push(arrowhead(a_line, d, style));
                out.arrows.push(arrowhead(b_line, d.scale(-1.0), style));
                out.polylines.push(vec![a3(a_line.sub(d.scale(style.arrow * 1.5))), a3(a_line)]);
                out.polylines.push(vec![a3(b_line), a3(b_line.add(d.scale(style.arrow * 1.5)))]);
            }
        }
        PmiGeometry::Radial {
            center,
            axis,
            radius,
            diameter,
            sphere,
        } => {
            let center = project(v3(*center));
            let axis = if *sphere || plane.is_some() { view } else { v3(*axis) };
            let u = perpendicular_in_plane(axis, label.sub(center));
            let p = center.add(u.scale(*radius));
            let outside = label.sub(center).length() > *radius;
            if *diameter {
                let q = center.sub(u.scale(*radius));
                out.polylines.push(vec![a3(q), a3(p)]);
                out.arrows.push(arrowhead(p, u, style));
                out.arrows.push(arrowhead(q, u.scale(-1.0), style));
                if outside {
                    out.polylines.push(vec![a3(p), a3(label)]);
                }
            } else {
                if outside {
                    out.polylines.push(vec![a3(label), a3(p)]);
                    out.arrows.push(arrowhead(p, u.scale(-1.0), style));
                } else {
                    out.polylines.push(vec![a3(center), a3(p)]);
                    out.arrows.push(arrowhead(p, u, style));
                }
            }
        }
        PmiGeometry::Angular {
            vertex,
            dir_a,
            dir_b,
            axis,
            degrees,
        } => {
            let vertex = project(v3(*vertex));
            let axis = v3(*axis);
            let dir_a = v3(*dir_a);
            let dir_b = v3(*dir_b);
            let rel = label.sub(vertex);
            let planar = rel.sub(axis.scale(rel.dot(axis)));
            let rho = planar.length().max(style.arrow * 2.0);
            let steps = ((degrees / 5.0).ceil() as usize).max(8);
            let mut arc = Vec::with_capacity(steps + 1);
            for k in 0..=steps {
                let angle = degrees.to_radians() * k as f64 / steps as f64;
                arc.push(a3(vertex.add(rotate(dir_a, axis, angle).scale(rho))));
            }
            let start = v3(arc[0]);
            let end = v3(arc[steps]);
            out.polylines.push(arc);
            let overshoot = rho + style.arrow * 0.6;
            out.polylines.push(vec![a3(vertex), a3(vertex.add(dir_a.scale(overshoot)))]);
            out.polylines.push(vec![a3(vertex), a3(vertex.add(dir_b.scale(overshoot)))]);
            let tangent_start = axis.cross(dir_a);
            let tangent_end = axis.cross(dir_b);
            out.arrows.push(arrowhead(start, tangent_start.scale(-1.0), style));
            out.arrows.push(arrowhead(end, tangent_end, style));
        }
        PmiGeometry::Leader { targets, dot: use_dot } => {
            for target in targets {
                let target = v3(*target);
                out.polylines.push(vec![a3(label), a3(target)]);
                if *use_dot {
                    out.polylines.push(dot(target, style));
                } else {
                    out.arrows.push(arrowhead(target, target.sub(label), style));
                }
            }
        }
        PmiGeometry::Hole { anchor, .. } => {
            let anchor = v3(*anchor);
            out.polylines.push(vec![a3(label), a3(anchor)]);
            out.arrows.push(arrowhead(anchor, anchor.sub(label), style));
        }
        PmiGeometry::Explode {
            center,
            translate,
            trace,
            ..
        } => {
            if *trace {
                let from = v3(*center);
                let to = from.add(v3(*translate));
                let span = to.sub(from);
                let length = span.length();
                if length > 1e-9 {
                    let dash = (style.arrow * 1.5).max(1e-6);
                    let d = span.scale(1.0 / length);
                    let mut s = 0.0;
                    while s < length {
                        let e = (s + dash).min(length);
                        out.polylines.push(vec![a3(from.add(d.scale(s))), a3(from.add(d.scale(e)))]);
                        s += dash * 2.0;
                    }
                }
            }
        }
        PmiGeometry::Datum { anchor, letter, .. } => {
            let anchor = v3(*anchor);
            let dir = unit_or(label.sub(anchor), up);
            // The datum triangle: base ON the feature, apex toward the label.
            let apex = anchor.add(dir.scale(style.arrow));
            let w = arrow_width_dir(dir, style);
            let half = style.arrow * 0.5;
            out.arrows.push([a3(apex), a3(anchor.add(w.scale(half))), a3(anchor.sub(w.scale(half)))]);
            out.polylines.push(vec![a3(apex), a3(label)]);
            let side = style.text_height * 1.8;
            let width = side.max(style.text_height * font::ADVANCE * letter.chars().count() as f64 + style.text_height);
            out.frames.push(frame_rect(label.sub(right.scale(width * 0.5)), width, side, right, up));
            out.texts.push(TextRun {
                text: letter.clone(),
                anchor: a3(label),
                right: a3(right),
                up: a3(up),
                height: style.text_height,
            });
        }
        PmiGeometry::Fcf { anchor, frame, .. } => {
            let anchor = v3(*anchor);
            out.polylines.push(vec![a3(label), a3(anchor)]);
            out.arrows.push(arrowhead(anchor, anchor.sub(label), style));
            let height = style.text_height * 1.8;
            let char_w = style.text_height * font::ADVANCE;
            let mut cells: Vec<(f64, &str)> = vec![(height, frame.symbol.as_str())];
            cells.push((char_w * frame.zone.chars().count() as f64 + style.text_height, frame.zone.as_str()));
            for datum in &frame.datums {
                cells.push((char_w * datum.chars().count() as f64 + style.text_height, datum.as_str()));
            }
            let total: f64 = cells.iter().map(|(w, _)| w).sum();
            let mut left = label.sub(right.scale(total * 0.5));
            for (width, cell_text) in cells {
                out.frames.push(frame_rect(left, width, height, right, up));
                out.texts.push(TextRun {
                    text: cell_text.to_string(),
                    anchor: a3(left.add(right.scale(width * 0.5))),
                    right: a3(right),
                    up: a3(up),
                    height: style.text_height,
                });
                left = left.add(right.scale(width));
            }
        }
    }
    // Every other annotation reads as one run centred on its label.
    if out.texts.is_empty() && !text.is_empty() {
        out.texts.push(TextRun {
            text: text.to_string(),
            anchor: a3(label),
            right: a3(right),
            up: a3(up),
            height: style.text_height,
        });
    }
    out
}
