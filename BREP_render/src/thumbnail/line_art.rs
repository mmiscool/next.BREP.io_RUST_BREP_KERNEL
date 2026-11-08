//! The Line art step picture: a flat technical illustration, as assembly
//! instructions draw a step (the instructions documents' default style).
//!
//! * a light grey ground ([`BACKGROUND`]);
//! * the parts earlier steps placed: flat white, thin grey visible edges and
//!   a darker grey outline;
//! * the parts the step adds: flat blue ([`ADDING_FILL`]) under a heavy black
//!   outline and thin black visible edges, so the new part pops;
//! * no lighting at all;
//! * black insertion arrows over everything.
//!
//! The outline is every part's silhouette as it shows: a sample whose
//! neighbour is another part, the ground, or the same part much further
//! away (a fold of the part over itself) is on an outline, drawn by the part
//! in front. It is found in the raster (a part-id buffer beside the depth
//! buffer), so a cylinder's outline is drawn although no B-rep edge runs
//! there. Edges are depth-tested against the faces, so hidden lines are
//! never drawn.

use super::{dot, f64s, normalize, Basis, Body, Projection, SUPERSAMPLE};

/// The ground (sRGB 0..1): `#ececec`, the reference frames' background.
pub const BACKGROUND: [f32; 3] = [0.925, 0.925, 0.925];
/// A placed part's fill: white.
pub const PLACED_FILL: [f32; 3] = [1.0, 1.0, 1.0];
/// A placed part's visible edges: light grey.
pub const PLACED_EDGE: [f32; 3] = [0.55, 0.55, 0.55];
/// A placed part's outline: dark grey.
pub const PLACED_OUTLINE: [f32; 3] = [0.30, 0.30, 0.30];
/// An added part's fill: `#9db8f5`, sampled from the reference frames.
pub const ADDING_FILL: [f32; 3] = [0.616, 0.722, 0.961];
/// An added part's visible edges and outline, and the arrows: black.
pub const INK: [f32; 3] = [0.0, 0.0, 0.0];

/// Line widths in output pixels at an 800-pixel picture (the reference
/// frames' proportions); a picture of another size scales them by the
/// square root of its size, so a small picture keeps readable lines.
const REFERENCE_PX: f64 = 800.0;
const EDGE_PX: f64 = 1.1;
const PLACED_OUTLINE_PX: f64 = 2.0;
const ADDING_OUTLINE_PX: f64 = 3.2;
/// The arrow: a solid shaft and a filled triangular head at the seat
/// (the reference's insertion frame at 900 px: a ~3 px line, a head ~22 px
/// long and ~16 px wide).
pub const ARROW_SHAFT_PX: f64 = 2.6;
pub const ARROW_HEAD_LENGTH_PX: f64 = 20.0;
pub const ARROW_HEAD_WIDTH_PX: f64 = 14.0;

/// The fraction of the picture the step (parts and arrows) fills.
const FILL: f64 = 0.86;

/// What one step picture draws.
#[derive(Debug, Clone, Default)]
pub struct LineArt {
    /// The parts earlier steps placed, each its own outline.
    pub placed: Vec<Body>,
    /// The parts the step adds.
    pub adding: Vec<Body>,
    /// Insertion arrows in world space: (tail, tip).
    pub arrows: Vec<([f64; 3], [f64; 3])>,
}

impl LineArt {
    fn is_empty(&self) -> bool {
        self.placed.iter().chain(&self.adding).all(|b| b.indices.is_empty())
    }
}

/// The picture as a `size`-pixel square PNG seen along `basis`, fit to the
/// parts and the arrows. `None` when there is nothing to draw.
pub fn render_png(art: &LineArt, size: u32, basis: Basis) -> Option<Vec<u8>> {
    let rgba = render_rgba(art, size, basis)?;
    super::encode_png(&rgba, size, size).ok()
}

/// [`render_png`]'s pixels: `size`² RGBA, row-major, opaque.
pub fn render_rgba(art: &LineArt, size: u32, basis: Basis) -> Option<Vec<u8>> {
    if art.is_empty() || size == 0 {
        return None;
    }
    let n = (size * SUPERSAMPLE) as usize;
    let mut extra: Vec<[f64; 3]> = Vec::new();
    for (tail, tip) in &art.arrows {
        extra.push(*tail);
        extra.push(*tip);
    }
    let bodies: Vec<(&Body, bool)> = art.placed.iter().map(|b| (b, false)).chain(art.adding.iter().map(|b| (b, true))).collect();
    let projection = Projection::fit_points(
        bodies.iter().flat_map(|(b, _)| b.indices.iter().map(|&i| f64s(b.positions[i as usize]))).chain(extra.iter().copied()),
        n as u32,
        basis,
        FILL,
    );
    let scale = (size as f64 / REFERENCE_PX).sqrt().clamp(0.5, 2.0);
    // In samples, never thinner than one output pixel.
    let px = |output_px: f64| (output_px * scale).max(1.0) * SUPERSAMPLE as f64;

    // Faces: depth and the part's id (0 is the ground).
    let mut depth = vec![f32::INFINITY; n * n];
    let mut id = vec![0u32; n * n];
    for (k, (body, _)) in bodies.iter().enumerate() {
        let screen: Vec<[f64; 3]> = body.positions.iter().map(|&p| projection.project(f64s(p))).collect();
        for tri in body.indices.chunks_exact(3) {
            let p = [screen[tri[0] as usize], screen[tri[1] as usize], screen[tri[2] as usize]];
            fill_id(&mut depth, &mut id, n, p, k as u32 + 1);
        }
    }
    let adding = |part: u32| part > 0 && bodies[part as usize - 1].1;
    let mut color: Vec<[f32; 3]> = id
        .iter()
        .map(|&part| if part == 0 { BACKGROUND } else if adding(part) { ADDING_FILL } else { PLACED_FILL })
        .collect();

    // Visible edges, depth-tested with a bias so an edge on its own face wins.
    let bias = 2.5f32;
    let edge_radius = px(EDGE_PX) * 0.5;
    for (body, lit) in &bodies {
        let ink = if *lit { INK } else { PLACED_EDGE };
        for line in &body.edges {
            for seg in line.windows(2) {
                let (p, q) = (projection.project(f64s(seg[0])), projection.project(f64s(seg[1])));
                stroke(&mut color, n, p, q, edge_radius, ink, Some((&depth, bias)));
            }
        }
    }

    // Outlines: a sample on a part whose neighbour is the ground, another
    // part behind it, or the same part far behind it. The nearer side draws
    // it; the added parts' outlines go last, over the placed parts'.
    let span = depth.iter().filter(|z| z.is_finite()).fold((f32::INFINITY, f32::NEG_INFINITY), |(lo, hi), &z| (lo.min(z), hi.max(z)));
    let fold = ((span.1 - span.0).max(1e-6)) * 0.04;
    let on_outline = |i: usize, j: usize| -> bool {
        let (a, b) = (id[i], id[j]);
        a != 0 && (b == 0 || (a != b && depth[i] <= depth[j]) || (a == b && depth[j] - depth[i] > fold))
    };
    let mut rims: [Vec<usize>; 2] = [Vec::new(), Vec::new()];
    for y in 0..n {
        for x in 0..n {
            let i = y * n + x;
            if id[i] == 0 {
                continue;
            }
            let edge = (x + 1 < n && on_outline(i, i + 1))
                || (x > 0 && on_outline(i, i - 1))
                || (y + 1 < n && on_outline(i, i + n))
                || (y > 0 && on_outline(i, i - n))
                || x == 0
                || y == 0
                || x + 1 == n
                || y + 1 == n;
            if edge {
                rims[adding(id[i]) as usize].push(i);
            }
        }
    }
    for (lit, rim) in rims.iter().enumerate() {
        let (radius, ink) = if lit == 1 { (px(ADDING_OUTLINE_PX) * 0.5, INK) } else { (px(PLACED_OUTLINE_PX) * 0.5, PLACED_OUTLINE) };
        let stamp = disc(radius);
        for &i in rim {
            let (x, y) = ((i % n) as i64, (i / n) as i64);
            for &(dx, dy) in &stamp {
                let (sx, sy) = (x + dx, y + dy);
                if sx >= 0 && sy >= 0 && (sx as usize) < n && (sy as usize) < n {
                    color[sy as usize * n + sx as usize] = ink;
                }
            }
        }
    }

    // Arrows over everything.
    for (tail, tip) in &art.arrows {
        let (a, b) = (projection.project(*tail), projection.project(*tip));
        arrow(&mut color, n, a, b, px(ARROW_SHAFT_PX) * 0.5, px(ARROW_HEAD_LENGTH_PX), px(ARROW_HEAD_WIDTH_PX) * 0.5);
    }

    // Box-filter down to the output size.
    let s = SUPERSAMPLE as usize;
    let out_w = size as usize;
    let mut out = vec![0u8; out_w * out_w * 4];
    let inv = 1.0 / (s * s) as f32;
    for y in 0..out_w {
        for x in 0..out_w {
            let mut acc = [0f32; 3];
            for dy in 0..s {
                for dx in 0..s {
                    let c = color[(y * s + dy) * n + x * s + dx];
                    for k in 0..3 {
                        acc[k] += c[k];
                    }
                }
            }
            let o = (y * out_w + x) * 4;
            for k in 0..3 {
                out[o + k] = ((acc[k] * inv).clamp(0.0, 1.0) * 255.0).round() as u8;
            }
            out[o + 3] = 255;
        }
    }
    Some(out)
}

/// The offsets of a filled disc of `radius` samples (at least the centre).
fn disc(radius: f64) -> Vec<(i64, i64)> {
    let r = radius.max(0.5);
    let reach = r.ceil() as i64;
    let mut out = Vec::new();
    for dy in -reach..=reach {
        for dx in -reach..=reach {
            if ((dx * dx + dy * dy) as f64) <= r * r + 0.25 {
                out.push((dx, dy));
            }
        }
    }
    out
}

/// Fill one screen triangle's depth and part id, depth-tested.
fn fill_id(depth: &mut [f32], id: &mut [u32], w: usize, p: [[f64; 3]; 3], part: u32) {
    let area = (p[1][0] - p[0][0]) * (p[2][1] - p[0][1]) - (p[2][0] - p[0][0]) * (p[1][1] - p[0][1]);
    if area.abs() < 1e-12 {
        return;
    }
    let min_x = p.iter().map(|q| q[0]).fold(f64::INFINITY, f64::min).floor().max(0.0) as usize;
    let max_x = (p.iter().map(|q| q[0]).fold(f64::NEG_INFINITY, f64::max).ceil() as i64).min(w as i64 - 1);
    let min_y = p.iter().map(|q| q[1]).fold(f64::INFINITY, f64::min).floor().max(0.0) as usize;
    let max_y = (p.iter().map(|q| q[1]).fold(f64::NEG_INFINITY, f64::max).ceil() as i64).min(w as i64 - 1);
    if max_x < 0 || max_y < 0 {
        return;
    }
    let inv_area = 1.0 / area;
    for y in min_y..=max_y as usize {
        let cy = y as f64 + 0.5;
        for x in min_x..=max_x as usize {
            let cx = x as f64 + 0.5;
            let w0 = ((p[1][0] - cx) * (p[2][1] - cy) - (p[2][0] - cx) * (p[1][1] - cy)) * inv_area;
            let w1 = ((p[2][0] - cx) * (p[0][1] - cy) - (p[0][0] - cx) * (p[2][1] - cy)) * inv_area;
            let w2 = 1.0 - w0 - w1;
            if w0 < 0.0 || w1 < 0.0 || w2 < 0.0 {
                continue;
            }
            let z = (w0 * p[0][2] + w1 * p[1][2] + w2 * p[2][2]) as f32;
            let i = y * w + x;
            if z < depth[i] {
                depth[i] = z;
                id[i] = part;
            }
        }
    }
}

/// A round-capped stroke `radius` samples wide from `p` to `q`; with
/// `depth`, only where the stroke is not behind a face (by more than the
/// bias).
fn stroke(color: &mut [[f32; 3]], w: usize, p: [f64; 3], q: [f64; 3], radius: f64, ink: [f32; 3], depth: Option<(&[f32], f32)>) {
    let steps = ((q[0] - p[0]).abs().max((q[1] - p[1]).abs()).ceil() as usize).max(1);
    let stamp = disc(radius);
    for i in 0..=steps {
        let t = i as f64 / steps as f64;
        let (x, y, z) = (p[0] + (q[0] - p[0]) * t, p[1] + (q[1] - p[1]) * t, (p[2] + (q[2] - p[2]) * t) as f32);
        let (cx, cy) = (x.floor() as i64, y.floor() as i64);
        if let Some((depth, bias)) = depth {
            if cx < 0 || cy < 0 || cx >= w as i64 || cy >= w as i64 {
                continue;
            }
            if z - bias > depth[cy as usize * w + cx as usize] {
                continue;
            }
        }
        for &(dx, dy) in &stamp {
            let (sx, sy) = (cx + dx, cy + dy);
            if sx >= 0 && sy >= 0 && sx < w as i64 && sy < w as i64 {
                color[sy as usize * w + sx as usize] = ink;
            }
        }
    }
}

/// A black arrow from `tail` to `tip` (screen samples): a shaft and a filled
/// head whose point is at `tip`.
fn arrow(color: &mut [[f32; 3]], w: usize, tail: [f64; 3], tip: [f64; 3], shaft: f64, head_length: f64, head_half_width: f64) {
    let (dx, dy) = (tip[0] - tail[0], tip[1] - tail[1]);
    let length = (dx * dx + dy * dy).sqrt();
    if length < 1.0 {
        return;
    }
    let (ux, uy) = (dx / length, dy / length);
    let head = head_length.min(length * 0.6);
    let base = [tip[0] - ux * head, tip[1] - uy * head, tip[2]];
    stroke(color, w, tail, base, shaft, INK, None);
    let (nx, ny) = (-uy * head_half_width, ux * head_half_width);
    let corners = [[tip[0], tip[1]], [base[0] + nx, base[1] + ny], [base[0] - nx, base[1] - ny]];
    let min_x = corners.iter().map(|c| c[0]).fold(f64::INFINITY, f64::min).floor().max(0.0) as i64;
    let max_x = (corners.iter().map(|c| c[0]).fold(f64::NEG_INFINITY, f64::max).ceil() as i64).min(w as i64 - 1);
    let min_y = corners.iter().map(|c| c[1]).fold(f64::INFINITY, f64::min).floor().max(0.0) as i64;
    let max_y = (corners.iter().map(|c| c[1]).fold(f64::NEG_INFINITY, f64::max).ceil() as i64).min(w as i64 - 1);
    let side = |a: [f64; 2], b: [f64; 2], p: [f64; 2]| (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0]);
    let orientation = side(corners[0], corners[1], corners[2]).signum();
    for y in min_y..=max_y {
        for x in min_x..=max_x {
            let p = [x as f64 + 0.5, y as f64 + 0.5];
            let inside = (0..3).all(|k| side(corners[k], corners[(k + 1) % 3], p) * orientation >= 0.0);
            if inside {
                color[y as usize * w + x as usize] = INK;
            }
        }
    }
}

/// A still's insertion arrow, in world space, for a part whose seated bounds
/// are `lo..hi` and whose motion starts `start_offset` away from home: from
/// its trailing face where the motion starts to just short of its trailing
/// face at home, so the arrow points at the seated part from where it came.
/// `None` for no motion.
pub fn still_arrow(lo: [f64; 3], hi: [f64; 3], start_offset: [f64; 3]) -> Option<([f64; 3], [f64; 3])> {
    let length = dot(start_offset, start_offset).sqrt();
    if !(length > 1e-12) {
        return None;
    }
    // The travel direction, start toward home.
    let d = normalize([-start_offset[0], -start_offset[1], -start_offset[2]]);
    let center = [(lo[0] + hi[0]) * 0.5, (lo[1] + hi[1]) * 0.5, (lo[2] + hi[2]) * 0.5];
    let half = [(hi[0] - lo[0]) * 0.5, (hi[1] - lo[1]) * 0.5, (hi[2] - lo[2]) * 0.5];
    // The bounds' half extent along d.
    let h = half[0] * d[0].abs() + half[1] * d[1].abs() + half[2] * d[2].abs();
    let trailing = [center[0] - d[0] * h, center[1] - d[1] * h, center[2] - d[2] * h];
    let tail = [trailing[0] + start_offset[0], trailing[1] + start_offset[1], trailing[2] + start_offset[2]];
    // Stop a little short of the part, so the head reads.
    let gap = (length * 0.06).min(h * 0.5);
    let tip = [trailing[0] - d[0] * gap, trailing[1] - d[1] * gap, trailing[2] - d[2] * gap];
    (dot([tip[0] - tail[0], tip[1] - tail[1], tip[2] - tail[2]], d) > 0.0).then_some((tail, tip))
}

