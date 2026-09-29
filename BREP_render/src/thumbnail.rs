//! Part thumbnails: a small CPU rasterizer over the [`RenderScene`]'s display
//! meshes, the one picture producer that runs the same on native, in the
//! browser and in the headless bake worker (which has no GPU device at all).
//!
//! The picture is the ViewCube's ISO view (from +X+Y+Z, +Y up), orthographic,
//! fit to what is drawn, square, on a transparent background: Lambert shading
//! under a fixed light, a depth buffer, each body's viewport colour (a face's
//! colour, else its solid's, else the name-hashed colour), and the display
//! edges drawn over it. It is supersampled [`SUPERSAMPLE`]× per axis and
//! averaged down, so silhouettes are anti-aliased.
//!
//! [`capture`] copies what the picture needs out of the scene (cheap, on the
//! caller's thread); [`render_png`] does the rest and is `Send`-safe, so a
//! native host can run it on a thread of its own. The PNG carries no text or
//! time chunks: the same scene gives the same bytes.

use crate::color::solid_color_srgb;
use crate::scene::RenderScene;

/// The renderer's version, stored beside each thumbnail on the server. Bump it
/// whenever the picture for the same document would change, so a server can
/// tell a thumbnail made by an older renderer.
pub const RENDERER: &str = "brep-thumb/1";

/// The default thumbnail edge, in pixels.
pub const SIZE: u32 = 256;

/// Samples per pixel per axis.
const SUPERSAMPLE: u32 = 2;

/// The fraction of the picture the model's projected extent fills.
const FILL: f64 = 0.9;

/// Edge colour (sRGB 0..1).
const EDGE: [f32; 3] = [0.10, 0.11, 0.13];

/// One body's triangles, ready to draw.
#[derive(Debug, Clone, Default)]
pub struct Body {
    pub positions: Vec<[f32; 3]>,
    pub normals: Vec<[f32; 3]>,
    pub indices: Vec<u32>,
    /// sRGB colour per triangle.
    pub colors: Vec<[f32; 3]>,
    /// World polylines of the body's display edges.
    pub edges: Vec<Vec<[f32; 3]>>,
}

/// What [`render_png`] draws: an owned copy of the visible scene.
#[derive(Debug, Clone, Default)]
pub struct Capture {
    pub bodies: Vec<Body>,
}

impl Capture {
    pub fn triangles(&self) -> usize {
        self.bodies.iter().map(|b| b.indices.len() / 3).sum()
    }

    pub fn is_empty(&self) -> bool {
        self.triangles() == 0
    }
}

/// Copy the visible bodies out of `scene`: every visible solid and committed
/// sketch sheet, hidden faces left out, in the viewport's colours.
pub fn capture(scene: &RenderScene) -> Capture {
    let mut bodies = Vec::new();
    for solid in scene.solids().iter().filter(|s| s.visible) {
        let mesh = &solid.mesh;
        let base = solid.color_override.unwrap_or_else(|| solid_color_srgb(&solid.name).map(|c| c as f32));
        let mut body = Body { positions: mesh.positions.clone(), normals: mesh.normals.clone(), ..Body::default() };
        for (tri, idx) in mesh.indices.chunks_exact(3).enumerate() {
            let face_index = mesh.face_ids.get(tri).map(|&f| f as usize);
            if face_index.is_some_and(|f| !solid.visibility.is_face_visible(f)) {
                continue;
            }
            let face = face_index.and_then(|f| solid.faces.get(f));
            body.indices.extend_from_slice(idx);
            body.colors.push(face.and_then(|f| f.color_override).unwrap_or(base));
        }
        body.edges = solid
            .edges
            .iter()
            .enumerate()
            .filter(|(i, e)| !e.centerline && solid.visibility.is_edge_visible(*i))
            .map(|(_, e)| e.polyline.clone())
            .collect();
        if !body.indices.is_empty() {
            bodies.push(body);
        }
    }
    Capture { bodies }
}

/// The ISO view's orthonormal basis: right, up, and the direction from the
/// model toward the eye.
fn iso_basis() -> ([f64; 3], [f64; 3], [f64; 3]) {
    let k = 1.0 / 3f64.sqrt();
    let back = [k, k, k];
    let up_world = [0.0, 1.0, 0.0];
    let right = normalize(cross(up_world, back));
    let up = cross(back, right);
    (right, up, back)
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn normalize(v: [f64; 3]) -> [f64; 3] {
    let n = dot(v, v).sqrt();
    if n > 0.0 { [v[0] / n, v[1] / n, v[2] / n] } else { v }
}

fn f64s(p: [f32; 3]) -> [f64; 3] {
    [p[0] as f64, p[1] as f64, p[2] as f64]
}

/// World → the supersampled raster: `(x, y, depth)`, x right, y down, depth
/// growing away from the eye.
struct Projection {
    right: [f64; 3],
    up: [f64; 3],
    back: [f64; 3],
    center: [f64; 3],
    scale: f64,
    half: f64,
}

impl Projection {
    fn fit(capture: &Capture, pixels: u32) -> Self {
        let (right, up, back) = iso_basis();
        // Fit the projected extent of every vertex, not the bbox's corners, so
        // a long thin part fills the picture.
        let (mut lo, mut hi) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]);
        for body in &capture.bodies {
            for &i in &body.indices {
                let p = f64s(body.positions[i as usize]);
                let (x, y) = (dot(p, right), dot(p, up));
                lo = [lo[0].min(x), lo[1].min(y)];
                hi = [hi[0].max(x), hi[1].max(y)];
            }
        }
        let extent = (hi[0] - lo[0]).max(hi[1] - lo[1]).max(1e-12);
        let mid = [(lo[0] + hi[0]) * 0.5, (lo[1] + hi[1]) * 0.5];
        let center = [
            right[0] * mid[0] + up[0] * mid[1],
            right[1] * mid[0] + up[1] * mid[1],
            right[2] * mid[0] + up[2] * mid[1],
        ];
        Self { right, up, back, center, scale: pixels as f64 * FILL / extent, half: pixels as f64 * 0.5 }
    }

    fn project(&self, p: [f64; 3]) -> [f64; 3] {
        let d = [p[0] - self.center[0], p[1] - self.center[1], p[2] - self.center[2]];
        [self.half + dot(d, self.right) * self.scale, self.half - dot(d, self.up) * self.scale, -dot(d, self.back) * self.scale]
    }
}

/// Render `capture` as a `size`×`size` RGBA PNG. `None` when nothing is drawn
/// (an empty document gets no thumbnail; consumers show their placeholder).
pub fn render_png(capture: &Capture, size: u32) -> Option<Vec<u8>> {
    let rgba = render_rgba(capture, size)?;
    encode_png(&rgba, size, size).ok()
}

/// [`render_png`]'s pixels, before encoding: `size`² RGBA, row-major.
pub fn render_rgba(capture: &Capture, size: u32) -> Option<Vec<u8>> {
    if capture.is_empty() || size == 0 {
        return None;
    }
    let n = size * SUPERSAMPLE;
    let w = n as usize;
    let projection = Projection::fit(capture, n);
    let mut depth = vec![f32::INFINITY; w * w];
    let mut color = vec![[0f32; 4]; w * w];
    // The key light, from the upper left of the view and toward the viewer; a
    // fill from the opposite side keeps faces turned away from it readable.
    let (right, up, back) = (projection.right, projection.up, projection.back);
    let key = normalize([
        -0.45 * right[0] + 0.6 * up[0] + 0.66 * back[0],
        -0.45 * right[1] + 0.6 * up[1] + 0.66 * back[1],
        -0.45 * right[2] + 0.6 * up[2] + 0.66 * back[2],
    ]);
    for body in &capture.bodies {
        let screen: Vec<[f64; 3]> = body.positions.iter().map(|&p| projection.project(f64s(p))).collect();
        for (tri, idx) in body.indices.chunks_exact(3).enumerate() {
            let [a, b, c] = [idx[0] as usize, idx[1] as usize, idx[2] as usize];
            let (pa, pb, pc) = (screen[a], screen[b], screen[c]);
            let normal_at = |i: usize| -> [f64; 3] {
                match body.normals.get(i) {
                    Some(&n) if n != [0.0; 3] => normalize(f64s(n)),
                    _ => {
                        let (wa, wb, wc) = (f64s(body.positions[a]), f64s(body.positions[b]), f64s(body.positions[c]));
                        normalize(cross([wb[0] - wa[0], wb[1] - wa[1], wb[2] - wa[2]], [wc[0] - wa[0], wc[1] - wa[1], wc[2] - wa[2]]))
                    }
                }
            };
            let shade = |n: [f64; 3]| -> f32 {
                // Two-sided: a normal facing away from the eye is flipped.
                let n = if dot(n, back) < 0.0 { [-n[0], -n[1], -n[2]] } else { n };
                (0.34 + 0.56 * dot(n, key).max(0.0) + 0.10 * dot(n, back).max(0.0)) as f32
            };
            let (sa, sb, sc) = (shade(normal_at(a)), shade(normal_at(b)), shade(normal_at(c)));
            let rgb = body.colors[tri];
            fill_triangle(&mut depth, &mut color, w, [pa, pb, pc], [sa, sb, sc], rgb);
        }
    }
    // Edges over the faces, depth-tested with a bias of a couple of samples so
    // an edge lying on its face wins.
    let bias = 2.5f32;
    for body in &capture.bodies {
        for line in &body.edges {
            for seg in line.windows(2) {
                let (p, q) = (projection.project(f64s(seg[0])), projection.project(f64s(seg[1])));
                draw_segment(&mut depth, &mut color, w, p, q, bias);
            }
        }
    }
    // Box-filter down to the output size.
    let s = SUPERSAMPLE as usize;
    let out_w = size as usize;
    let mut out = vec![0u8; out_w * out_w * 4];
    let inv = 1.0 / (s * s) as f32;
    for y in 0..out_w {
        for x in 0..out_w {
            let mut acc = [0f32; 4];
            for dy in 0..s {
                for dx in 0..s {
                    let c = color[(y * s + dy) * w + x * s + dx];
                    // Premultiplied accumulation.
                    for k in 0..3 {
                        acc[k] += c[k] * c[3];
                    }
                    acc[3] += c[3];
                }
            }
            let alpha = acc[3] * inv;
            let o = (y * out_w + x) * 4;
            if acc[3] > 0.0 {
                for k in 0..3 {
                    out[o + k] = ((acc[k] / acc[3]).clamp(0.0, 1.0) * 255.0).round() as u8;
                }
            }
            out[o + 3] = (alpha.clamp(0.0, 1.0) * 255.0).round() as u8;
        }
    }
    Some(out)
}

/// Fill one screen triangle with Gouraud-interpolated shade, depth-tested.
fn fill_triangle(depth: &mut [f32], color: &mut [[f32; 4]], w: usize, p: [[f64; 3]; 3], shade: [f32; 3], rgb: [f32; 3]) {
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
            if z >= depth[i] {
                continue;
            }
            depth[i] = z;
            let s = w0 as f32 * shade[0] + w1 as f32 * shade[1] + w2 as f32 * shade[2];
            color[i] = [rgb[0] * s, rgb[1] * s, rgb[2] * s, 1.0];
        }
    }
}

/// Draw one edge segment, about 1.5 output pixels wide, where it is not hidden.
fn draw_segment(depth: &mut [f32], color: &mut [[f32; 4]], w: usize, p: [f64; 3], q: [f64; 3], bias: f32) {
    let steps = ((q[0] - p[0]).abs().max((q[1] - p[1]).abs()).ceil() as usize).max(1);
    for i in 0..=steps {
        let t = i as f64 / steps as f64;
        let (x, y, z) = (p[0] + (q[0] - p[0]) * t, p[1] + (q[1] - p[1]) * t, (p[2] + (q[2] - p[2]) * t) as f32);
        for (dx, dy) in [(0i64, 0i64), (1, 0), (0, 1), (1, 1)] {
            let (px, py) = (x.floor() as i64 + dx, y.floor() as i64 + dy);
            if px < 0 || py < 0 || px >= w as i64 || py >= w as i64 {
                continue;
            }
            let k = py as usize * w + px as usize;
            if z - bias <= depth[k] {
                color[k] = [EDGE[0], EDGE[1], EDGE[2], 1.0];
            }
        }
    }
}

/// 8-bit RGBA PNG, no ancillary chunks.
fn encode_png(rgba: &[u8], width: u32, height: u32) -> Result<Vec<u8>, String> {
    let mut out = Vec::new();
    {
        let mut encoder = png::Encoder::new(&mut out, width, height);
        encoder.set_color(png::ColorType::Rgba);
        encoder.set_depth(png::BitDepth::Eight);
        let mut writer = encoder.write_header().map_err(|error| format!("png header: {error}"))?;
        writer.write_image_data(rgba).map_err(|error| format!("png data: {error}"))?;
    }
    Ok(out)
}

