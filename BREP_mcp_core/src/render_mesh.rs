//! CPU mesh renderer and silhouette comparison.
//!
//! Renders a triangle mesh `{positions, indices}` to a PNG with a fixed
//! isometric, Z-up camera, and measures the two shape statistics the
//! step-validation gallery is built on:
//!
//! * **silhouette IoU** — how much two engines' foreground pixels agree, and
//! * **inside-out fraction** — how much of each render is back-facing (red).
//!
//! There is no GPU, no window and no browser: triangles are rasterised into a
//! supersampled colour/depth buffer on the CPU and box-resolved to the output
//! image, which is what a multisampled GL framebuffer resolves to as well.
//!
//! # Matching the browser renderer
//!
//! Everything the two statistics depend on is reproduced exactly:
//!
//! * the three framing modes ([`Framing`]) and the camera built from them,
//! * the front/back split — a triangle is "back" when its *screen-space*
//!   winding is clockwise, and back faces are unlit pure red, so an inside-out
//!   solid (or the far interior wall seen through a hole) reads red,
//!   just as the two-material `FrontSide`/`BackSide` pair does in three.js,
//! * the classification thresholds [`ALPHA_OPAQUE`], [`RED_MIN`], [`RED_RATIO`],
//! * feature edges: `EdgesGeometry(geo, 30)` — the same position hashing,
//!   dihedral test and 2 px screen-space width.
//!
//! What is deliberately *not* reproduced is the front faces' shading:
//! three.js uses `MeshStandardMaterial` (GGX) under a hemisphere plus a
//! directional light, and this renderer uses Lambert plus a hemisphere ambient
//! in linear space with an sRGB encode. Front shading cannot change either
//! statistic — the silhouette does not depend on colour, and the front colours
//! the harness uses never satisfy the red test — so the approximation is
//! confined to how the picture looks.
//!
//! # Alpha convention
//!
//! The browser's in-page sampler reads `gl.readPixels`, which is
//! **premultiplied**: a boundary pixel covered by a fraction `c` of red reads
//! `(255c, 0, 0, 255c)`, so [`RED_MIN`] is a test on the *premultiplied* red.
//! `toDataURL` un-premultiplies, so the PNG holds straight alpha. This module
//! writes straight-alpha PNGs (the colour is the mean over covered samples) and
//! [`silhouette_of`] premultiplies before applying [`RED_MIN`], so a count taken
//! from a PNG equals the count the browser took from its framebuffer.
use crate::image::encode_png;
use image::{ImageBuffer, Rgba, RgbaImage};
use std::collections::HashMap;

// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------

/// A pixel counts as opaque (foreground) when its alpha exceeds this.
pub const ALPHA_OPAQUE: u8 = 8;
/// Red must exceed green and blue by this factor to read as a back face.
pub const RED_RATIO: f64 = 1.6;
/// ...and the premultiplied red must be at least this bright.
pub const RED_MIN: f64 = 40.0;

/// Vertical field of view, degrees.
pub const FOV_DEG: f64 = 35.0;
/// Near plane, as a multiple of the framing radius.
pub const NEAR_FACTOR: f64 = 0.01;
/// Far plane, as a multiple of the framing radius.
pub const FAR_FACTOR: f64 = 200.0;
/// Slack on the fitted camera distance.
pub const FIT_MARGIN: f64 = 1.08;
/// Camera offset direction from the framing centre (normalised on use).
pub const EYE_DIR: [f64; 3] = [1.0, 0.85, 0.75];
/// Camera up. Z-up, reproducing the retired three.js gallery renderer this
/// replaced (and the convention the STEP corpus it renders is authored in) —
/// not the application's own world, which is +Y up. Both sides of a silhouette
/// comparison are framed by this camera, so the statistics do not care; changing
/// it would only invalidate the gallery's existing images.
pub const CAMERA_UP: [f64; 3] = [0.0, 0.0, 1.0];
/// Directional light offset from the framing centre, times `radius * 4`.
pub const LIGHT_DIR: [f64; 3] = [1.0, 1.4, 1.0];
/// Directional light intensity.
pub const LIGHT_INTENSITY: f64 = 1.3;
/// Hemisphere light sky colour (sRGB).
pub const HEMI_SKY: [u8; 3] = [0xff, 0xff, 0xff];
/// Hemisphere light ground colour (sRGB).
pub const HEMI_GROUND: [u8; 3] = [0x20, 0x24, 0x30];
/// Hemisphere light intensity.
pub const HEMI_INTENSITY: f64 = 1.0;
/// Hemisphere light axis. Three.js's default light up is +Y, not the camera up.
pub const HEMI_UP: [f64; 3] = [0.0, 1.0, 0.0];
/// Back faces are drawn in this unlit colour.
pub const BACK_FACE_COLOR: [u8; 3] = [0xff, 0x00, 0x00];
/// Default front colour (`renderScene`'s own default).
pub const DEFAULT_COLOR: [u8; 3] = [0x9d, 0xb4, 0xd4];
/// Default feature-edge colour (`renderScene`'s own default).
pub const DEFAULT_EDGE_COLOR: [u8; 3] = [0x22, 0x30, 0x44];
/// Output width in pixels.
pub const DEFAULT_WIDTH: u32 = 960;
/// Output height in pixels.
pub const DEFAULT_HEIGHT: u32 = 720;
/// Supersampling factor per axis (so `samples²` samples per output pixel).
pub const DEFAULT_SAMPLES: u32 = 4;
/// Dihedral threshold for a feature edge, degrees — `EdgesGeometry(geo, 30)`.
pub const EDGE_ANGLE_DEG: f64 = 30.0;
/// Feature-edge width in output pixels — `LineMaterial({ linewidth: 2 })`.
pub const EDGE_WIDTH_PX: f64 = 2.0;
/// Vertex-merge precision for the feature-edge pass: three.js's `EdgesGeometry`
/// hashes positions rounded to four decimal places.
const EDGE_HASH_PRECISION: f64 = 1e4;
/// Depth slack, as a fraction of the eye distance, that lets a feature edge win
/// against the surface it lies on. Three.js gets the same effect for free: its
/// screen-space line quads keep the edge's own depth while stepping sideways
/// onto a face that slopes away.
const EDGE_DEPTH_SLACK: f64 = 2e-3;

// ---------------------------------------------------------------------------
// Input
// ---------------------------------------------------------------------------

/// A triangle soup: `positions` is xyz-interleaved, `indices` is empty for a
/// non-indexed mesh (every three positions is one triangle).
#[derive(Clone, Debug, Default)]
pub struct Mesh {
    pub positions: Vec<f64>,
    pub indices: Vec<u32>,
}

impl Mesh {
    /// Read the `{positions, indices}` JSON the harness's `step_import_batch`
    /// and `occStepToMesh` both emit. Extra keys (`triangles`, `error`, …) are
    /// ignored.
    pub fn from_json(value: &serde_json::Value) -> Result<Mesh, String> {
        let obj = value.as_object().ok_or("mesh json: expected an object")?;
        let nums = |key: &str| -> Result<Vec<f64>, String> {
            match obj.get(key) {
                None | Some(serde_json::Value::Null) => Ok(Vec::new()),
                Some(serde_json::Value::Array(a)) => a
                    .iter()
                    .map(|v| v.as_f64().ok_or_else(|| format!("mesh json: {key} holds a non-number")))
                    .collect(),
                Some(_) => Err(format!("mesh json: {key} is not an array")),
            }
        };
        let positions = nums("positions")?;
        if positions.len() % 3 != 0 {
            return Err(format!("mesh json: positions length {} is not a multiple of 3", positions.len()));
        }
        let indices: Vec<u32> = nums("indices")?.iter().map(|v| *v as u32).collect();
        if indices.len() % 3 != 0 {
            return Err(format!("mesh json: indices length {} is not a multiple of 3", indices.len()));
        }
        Ok(Mesh { positions, indices })
    }

    /// Parse the JSON text of a mesh file.
    pub fn from_json_str(text: &str) -> Result<Mesh, String> {
        let value: serde_json::Value =
            serde_json::from_str(text).map_err(|e| format!("mesh json: {e}"))?;
        Mesh::from_json(&value)
    }

    /// Vertices as points, and triangles as index triples (synthesised for a
    /// non-indexed mesh).
    fn unpack(&self) -> (Vec<[f64; 3]>, Vec<[u32; 3]>) {
        let verts: Vec<[f64; 3]> =
            self.positions.chunks_exact(3).map(|c| [c[0], c[1], c[2]]).collect();
        let tris: Vec<[u32; 3]> = if self.indices.is_empty() {
            (0..verts.len() / 3).map(|t| [(t * 3) as u32, (t * 3 + 1) as u32, (t * 3 + 2) as u32]).collect()
        } else {
            self.indices
                .chunks_exact(3)
                .filter(|c| c.iter().all(|i| (*i as usize) < verts.len()))
                .map(|c| [c[0], c[1], c[2]])
                .collect()
        };
        (verts, tris)
    }
}

/// How the camera is placed relative to the mesh.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Framing {
    /// Frame the camera to this mesh's own bounding box.
    Auto,
    /// An explicit camera override.
    Explicit { center: [f64; 3], radius: f64 },
    /// Recentre and rescale the mesh onto the canonical unit sphere, then frame
    /// that. Two meshes of the same shape then land on identical pixels even
    /// when the engines emit different unit scales — which is what makes their
    /// silhouettes comparable pixel for pixel.
    Normalize,
}

impl Default for Framing {
    fn default() -> Self {
        Framing::Auto
    }
}

/// Everything `renderScene(mesh, opts)` took.
#[derive(Clone, Debug)]
pub struct RenderOptions {
    pub width: u32,
    pub height: u32,
    pub color: [u8; 3],
    pub edges: bool,
    pub edge_color: [u8; 3],
    pub framing: Framing,
    /// Supersampling factor per axis; `samples * samples` samples per pixel.
    pub samples: u32,
}

impl Default for RenderOptions {
    fn default() -> Self {
        RenderOptions {
            width: DEFAULT_WIDTH,
            height: DEFAULT_HEIGHT,
            color: DEFAULT_COLOR,
            edges: true,
            edge_color: DEFAULT_EDGE_COLOR,
            framing: Framing::Auto,
            samples: DEFAULT_SAMPLES,
        }
    }
}

// ---------------------------------------------------------------------------
// Output
// ---------------------------------------------------------------------------

/// The pixel classification of one render: which pixels are foreground, how
/// many there are, and how many of those are back-facing.
#[derive(Clone, Debug)]
pub struct Silhouette {
    /// One byte per pixel, 1 where the pixel is opaque.
    pub mask: Vec<u8>,
    pub width: u32,
    pub height: u32,
    pub opaque: u64,
    pub red: u64,
}

impl Silhouette {
    /// Back-facing pixels over foreground pixels — the inside-out fraction.
    pub fn backface_fraction(&self) -> f64 {
        if self.opaque == 0 {
            0.0
        } else {
            self.red as f64 / self.opaque as f64
        }
    }
}

/// One rendered mesh.
pub struct Render {
    pub image: RgbaImage,
    pub silhouette: Silhouette,
}

impl Render {
    /// The image as PNG bytes.
    pub fn png(&self) -> Result<Vec<u8>, String> {
        encode_png(&self.image)
    }
}

/// Two silhouettes measured against each other. `iou` and `mismatch` are the
/// gallery's shape-agreement metric; `backface_*` is each engine's inside-out
/// fraction.
#[derive(Clone, Copy, Debug)]
pub struct Comparison {
    pub opaque_a: u64,
    pub opaque_b: u64,
    pub red_a: u64,
    pub red_b: u64,
    pub intersection: u64,
    pub union: u64,
    pub iou: f64,
    pub mismatch: f64,
    pub backface_a: f64,
    pub backface_b: f64,
}

// ---------------------------------------------------------------------------
// Classification
// ---------------------------------------------------------------------------

/// Classify a straight-alpha RGBA image the way the browser's in-page sampler
/// classified its premultiplied framebuffer.
pub fn silhouette_of(img: &RgbaImage) -> Silhouette {
    let (width, height) = img.dimensions();
    let mut mask = vec![0u8; (width as usize) * (height as usize)];
    let (mut opaque, mut red) = (0u64, 0u64);
    for (i, px) in img.pixels().enumerate() {
        let [r, g, b, a] = px.0;
        if a <= ALPHA_OPAQUE {
            continue;
        }
        mask[i] = 1;
        opaque += 1;
        // Premultiply before the brightness test: the browser read premultiplied
        // pixels, so a barely-covered fringe pixel had a dim red, not a full one.
        let cover = a as f64 / 255.0;
        if (r as f64) * cover > RED_MIN
            && (r as f64) > (g as f64) * RED_RATIO
            && (r as f64) > (b as f64) * RED_RATIO
        {
            red += 1;
        }
    }
    Silhouette { mask, width, height, opaque, red }
}

/// Measure two silhouettes against each other. Pixels beyond the shorter mask
/// are ignored, as the browser comparison did.
pub fn compare(a: &Silhouette, b: &Silhouette) -> Comparison {
    let n = a.mask.len().min(b.mask.len());
    let (mut intersection, mut union) = (0u64, 0u64);
    for i in 0..n {
        let (x, y) = (a.mask[i], b.mask[i]);
        if x != 0 && y != 0 {
            intersection += 1;
        }
        if x != 0 || y != 0 {
            union += 1;
        }
    }
    let (iou, mismatch) = if union > 0 {
        (intersection as f64 / union as f64, (union - intersection) as f64 / union as f64)
    } else {
        (1.0, 0.0)
    };
    Comparison {
        opaque_a: a.opaque,
        opaque_b: b.opaque,
        red_a: a.red,
        red_b: b.red,
        intersection,
        union,
        iou,
        mismatch,
        backface_a: a.backface_fraction(),
        backface_b: b.backface_fraction(),
    }
}

/// Render both meshes into one shared canonical frame ([`Framing::Normalize`])
/// and measure them — the native `compareSilhouettes`.
pub fn compare_meshes(
    mesh_a: &Mesh,
    mesh_b: &Mesh,
    opts_a: &RenderOptions,
    opts_b: &RenderOptions,
) -> Result<(Render, Render, Comparison), String> {
    let a = render(mesh_a, &RenderOptions { framing: Framing::Normalize, ..opts_a.clone() })?;
    let b = render(mesh_b, &RenderOptions { framing: Framing::Normalize, ..opts_b.clone() })?;
    let cmp = compare(&a.silhouette, &b.silhouette);
    Ok((a, b, cmp))
}

// ---------------------------------------------------------------------------
// Linear algebra (4x4 column-major, as three.js and GL keep them)
// ---------------------------------------------------------------------------

type Mat4 = [f64; 16];

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn length(a: [f64; 3]) -> f64 {
    dot(a, a).sqrt()
}

fn normalize(a: [f64; 3]) -> [f64; 3] {
    let l = length(a);
    if l == 0.0 {
        [0.0, 0.0, 0.0]
    } else {
        [a[0] / l, a[1] / l, a[2] / l]
    }
}

fn scaled(a: [f64; 3], s: f64) -> [f64; 3] {
    [a[0] * s, a[1] * s, a[2] * s]
}

fn mat_mul(a: &Mat4, b: &Mat4) -> Mat4 {
    let mut out = [0.0; 16];
    for c in 0..4 {
        for r in 0..4 {
            let mut sum = 0.0;
            for k in 0..4 {
                sum += a[k * 4 + r] * b[c * 4 + k];
            }
            out[c * 4 + r] = sum;
        }
    }
    out
}

fn transform_point(m: &Mat4, p: [f64; 3]) -> [f64; 4] {
    [
        m[0] * p[0] + m[4] * p[1] + m[8] * p[2] + m[12],
        m[1] * p[0] + m[5] * p[1] + m[9] * p[2] + m[13],
        m[2] * p[0] + m[6] * p[1] + m[10] * p[2] + m[14],
        m[3] * p[0] + m[7] * p[1] + m[11] * p[2] + m[15],
    ]
}

/// `Matrix4.lookAt(eye, target, up)` inverted into a view matrix, exactly as
/// three.js builds a camera's `matrixWorldInverse`.
fn view_matrix(eye: [f64; 3], target: [f64; 3], up: [f64; 3]) -> Mat4 {
    let mut z = sub(eye, target);
    if length(z) == 0.0 {
        z = [0.0, 0.0, 1.0];
    }
    z = normalize(z);
    let mut x = cross(up, z);
    if length(x) == 0.0 {
        // The up vector is parallel to the view direction; nudge it, as three does.
        let nudged = if z[2].abs() == 1.0 { [up[0] + 1e-4, up[1], up[2]] } else { [up[0], up[1], up[2] + 1e-4] };
        x = cross(nudged, z);
    }
    x = normalize(x);
    let y = cross(z, x);
    // The inverse of the rigid basis [x y z | eye].
    [
        x[0], y[0], z[0], 0.0,
        x[1], y[1], z[1], 0.0,
        x[2], y[2], z[2], 0.0,
        -dot(x, eye), -dot(y, eye), -dot(z, eye), 1.0,
    ]
}

/// `PerspectiveCamera.updateProjectionMatrix` with the default zoom and no
/// film offset — the standard GL frustum.
fn perspective(fov_deg: f64, aspect: f64, near: f64, far: f64) -> Mat4 {
    let top = near * (fov_deg.to_radians() / 2.0).tan();
    let height = 2.0 * top;
    let width = aspect * height;
    let left = -0.5 * width;
    let right = left + width;
    let bottom = top - height;
    let x = 2.0 * near / (right - left);
    let y = 2.0 * near / (top - bottom);
    let a = (right + left) / (right - left);
    let b = (top + bottom) / (top - bottom);
    let c = -(far + near) / (far - near);
    let d = -2.0 * far * near / (far - near);
    [x, 0.0, 0.0, 0.0, 0.0, y, 0.0, 0.0, a, b, c, -1.0, 0.0, 0.0, d, 0.0]
}

// ---------------------------------------------------------------------------
// Colour
// ---------------------------------------------------------------------------

fn srgb_to_linear(c: f64) -> f64 {
    if c <= 0.04045 {
        c / 12.92
    } else {
        ((c + 0.055) / 1.055).powf(2.4)
    }
}

fn linear_to_srgb(c: f64) -> f64 {
    if c <= 0.0031308 {
        c * 12.92
    } else {
        1.055 * c.powf(1.0 / 2.4) - 0.055
    }
}

fn to_linear(c: [u8; 3]) -> [f64; 3] {
    [
        srgb_to_linear(c[0] as f64 / 255.0),
        srgb_to_linear(c[1] as f64 / 255.0),
        srgb_to_linear(c[2] as f64 / 255.0),
    ]
}

fn encode_srgb(c: [f64; 3]) -> [u8; 3] {
    let f = |v: f64| (linear_to_srgb(v.clamp(0.0, 1.0)) * 255.0).round().clamp(0.0, 255.0) as u8;
    [f(c[0]), f(c[1]), f(c[2])]
}

/// Parse `#rrggbb` / `rrggbb` / `0xrrggbb`.
pub fn parse_color(text: &str) -> Result<[u8; 3], String> {
    let t = text.trim().trim_start_matches('#').trim_start_matches("0x").trim_start_matches("0X");
    if t.len() != 6 || !t.chars().all(|c| c.is_ascii_hexdigit()) {
        return Err(format!("colour: expected #rrggbb, got {text:?}"));
    }
    let v = u32::from_str_radix(t, 16).map_err(|e| format!("colour {text:?}: {e}"))?;
    Ok([(v >> 16) as u8, (v >> 8) as u8, v as u8])
}

// ---------------------------------------------------------------------------
// Framebuffer
// ---------------------------------------------------------------------------

struct FrameBuffer {
    w: usize,
    h: usize,
    /// Per-sample colour; only meaningful where `depth` is finite.
    color: Vec<[u8; 3]>,
    /// Per-sample `1/w` (larger is nearer); `f32::NEG_INFINITY` where uncovered.
    depth: Vec<f32>,
}

impl FrameBuffer {
    fn new(w: usize, h: usize) -> FrameBuffer {
        FrameBuffer { w, h, color: vec![[0, 0, 0]; w * h], depth: vec![f32::NEG_INFINITY; w * h] }
    }

    /// Box-resolve the supersampled buffer, exactly as a multisampled GL
    /// framebuffer resolves against a fully transparent clear: the colour is
    /// the mean over *covered* samples and the alpha is the coverage, which is
    /// the straight-alpha form `toDataURL` writes.
    fn resolve(&self, width: u32, height: u32, s: usize) -> RgbaImage {
        let s2 = (s * s) as f64;
        ImageBuffer::from_fn(width, height, |px, py| {
            let (mut r, mut g, mut b, mut covered) = (0u32, 0u32, 0u32, 0u32);
            for dy in 0..s {
                let row = (py as usize * s + dy) * self.w;
                for dx in 0..s {
                    let i = row + px as usize * s + dx;
                    if self.depth[i] > f32::NEG_INFINITY {
                        let c = self.color[i];
                        r += c[0] as u32;
                        g += c[1] as u32;
                        b += c[2] as u32;
                        covered += 1;
                    }
                }
            }
            if covered == 0 {
                return Rgba([0, 0, 0, 0]);
            }
            let n = covered;
            let a = ((covered as f64) * 255.0 / s2).round().clamp(0.0, 255.0) as u8;
            Rgba([(r / n) as u8, (g / n) as u8, (b / n) as u8, a])
        })
    }
}

// ---------------------------------------------------------------------------
// The renderer
// ---------------------------------------------------------------------------

/// A screen-space vertex: pixel x/y in the supersampled buffer, `1/w`, and the
/// world normal already divided by `w` for perspective-correct interpolation.
#[derive(Clone, Copy)]
struct ScreenVertex {
    x: f64,
    y: f64,
    inv_w: f64,
    n_over_w: [f64; 3],
}

/// A clip-space vertex carried through the near-plane clip.
#[derive(Clone, Copy)]
struct ClipVertex {
    p: [f64; 4],
    n: [f64; 3],
}

fn lerp_clip(a: &ClipVertex, b: &ClipVertex, t: f64) -> ClipVertex {
    let mut p = [0.0; 4];
    for i in 0..4 {
        p[i] = a.p[i] + (b.p[i] - a.p[i]) * t;
    }
    let mut n = [0.0; 3];
    for i in 0..3 {
        n[i] = a.n[i] + (b.n[i] - a.n[i]) * t;
    }
    ClipVertex { p, n }
}

/// Render one mesh.
pub fn render(mesh: &Mesh, opts: &RenderOptions) -> Result<Render, String> {
    if opts.width == 0 || opts.height == 0 {
        return Err("render: zero-sized output".into());
    }
    let s = opts.samples.clamp(1, 8) as usize;
    let (mut verts, tris) = mesh.unpack();

    // --- framing: the three modes of render.mjs, unchanged.
    let (center, radius) = match opts.framing {
        Framing::Explicit { center, radius } => (center, radius.max(1e-9)),
        Framing::Normalize => {
            let (c, size) = bounding_box(&verts);
            let r = (length(size) / 2.0).max(1e-9);
            for v in verts.iter_mut() {
                for i in 0..3 {
                    v[i] = (v[i] - c[i]) / r;
                }
            }
            ([0.0, 0.0, 0.0], 1.0)
        }
        Framing::Auto => {
            let (c, size) = bounding_box(&verts);
            (c, (length(size) / 2.0).max(1e-9))
        }
    };

    // --- camera, matching render.mjs term for term.
    let near = radius * NEAR_FACTOR;
    let far = radius * FAR_FACTOR;
    let fit = radius / (FOV_DEG.to_radians() / 2.0).sin() * FIT_MARGIN;
    let eye = {
        let d = scaled(normalize(EYE_DIR), fit);
        [center[0] + d[0], center[1] + d[1], center[2] + d[2]]
    };
    let view = view_matrix(eye, center, CAMERA_UP);
    let proj = perspective(FOV_DEG, opts.width as f64 / opts.height as f64, near, far);
    let view_proj = mat_mul(&proj, &view);

    // --- the directional light sits at `center + normalize(LIGHT_DIR)*radius*4`
    // and aims at the world origin, which is where three.js leaves its target.
    let light_pos = {
        let d = scaled(normalize(LIGHT_DIR), radius * 4.0);
        [center[0] + d[0], center[1] + d[1], center[2] + d[2]]
    };
    let light = normalize(light_pos);

    let normals = vertex_normals(&verts, &tris);
    let albedo = to_linear(opts.color);
    let back = BACK_FACE_COLOR;

    let (bw, bh) = (opts.width as usize * s, opts.height as usize * s);
    let mut fb = FrameBuffer::new(bw, bh);

    for tri in &tris {
        let clip: Vec<ClipVertex> = tri
            .iter()
            .map(|&i| ClipVertex {
                p: transform_point(&view_proj, verts[i as usize]),
                n: normals[i as usize],
            })
            .collect();
        let poly = clip_near(&clip);
        if poly.len() < 3 {
            continue;
        }
        let screen: Vec<ScreenVertex> = poly.iter().map(|v| to_screen(v, bw, bh)).collect();

        // Facing comes from the winding of the whole projected polygon, so a
        // sliver produced by the clip cannot flip it. Screen y runs down, so a
        // GL front face (counter-clockwise with y up) has a negative area here.
        let mut area2 = 0.0;
        for i in 1..screen.len() - 1 {
            area2 += edge_fn(&screen[0], &screen[i], &screen[i + 1]);
        }
        if area2 == 0.0 {
            continue;
        }
        let front = area2 < 0.0;

        for i in 1..screen.len() - 1 {
            raster_triangle(
                &mut fb,
                [screen[0], screen[i], screen[i + 1]],
                front,
                albedo,
                back,
                light,
            );
        }
    }

    if opts.edges {
        draw_feature_edges(&mut fb, &verts, &tris, &view_proj, opts.edge_color, s);
    }

    let image = fb.resolve(opts.width, opts.height, s);
    let silhouette = silhouette_of(&image);
    Ok(Render { image, silhouette })
}

/// Centre and size of the axis-aligned bounding box, as `Box3.getCenter` /
/// `Box3.getSize` return them (an empty box collapses to the origin).
fn bounding_box(verts: &[[f64; 3]]) -> ([f64; 3], [f64; 3]) {
    if verts.is_empty() {
        return ([0.0; 3], [0.0; 3]);
    }
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    for v in verts {
        for i in 0..3 {
            lo[i] = lo[i].min(v[i]);
            hi[i] = hi[i].max(v[i]);
        }
    }
    let mut c = [0.0; 3];
    let mut size = [0.0; 3];
    for i in 0..3 {
        c[i] = (lo[i] + hi[i]) * 0.5;
        size[i] = hi[i] - lo[i];
    }
    (c, size)
}

/// `BufferGeometry.computeVertexNormals`: accumulate each face's un-normalised
/// cross product (so faces weigh by area) onto its three vertices, then
/// normalise.
fn vertex_normals(verts: &[[f64; 3]], tris: &[[u32; 3]]) -> Vec<[f64; 3]> {
    let mut normals = vec![[0.0f64; 3]; verts.len()];
    for t in tris {
        let (a, b, c) = (verts[t[0] as usize], verts[t[1] as usize], verts[t[2] as usize]);
        let n = cross(sub(b, a), sub(c, a));
        for &i in t {
            let acc = &mut normals[i as usize];
            for k in 0..3 {
                acc[k] += n[k];
            }
        }
    }
    for n in normals.iter_mut() {
        *n = normalize(*n);
    }
    normals
}

/// Clip a triangle against the near plane (`z >= -w` in clip space) so no
/// vertex reaches the projection with a non-positive `w`. Returns the clipped
/// convex polygon, orientation preserved.
fn clip_near(tri: &[ClipVertex]) -> Vec<ClipVertex> {
    let inside = |v: &ClipVertex| v.p[2] >= -v.p[3];
    if tri.iter().all(inside) {
        return tri.to_vec();
    }
    let mut out: Vec<ClipVertex> = Vec::with_capacity(4);
    for i in 0..tri.len() {
        let a = &tri[i];
        let b = &tri[(i + 1) % tri.len()];
        let (da, db) = (a.p[2] + a.p[3], b.p[2] + b.p[3]);
        let (ia, ib) = (da >= 0.0, db >= 0.0);
        if ia {
            out.push(*a);
        }
        if ia != ib {
            let denom = da - db;
            if denom.abs() > f64::MIN_POSITIVE {
                out.push(lerp_clip(a, b, da / denom));
            }
        }
    }
    out.retain(|v| v.p[3] > 0.0);
    out
}

/// Clip space to the supersampled pixel grid.
fn to_screen(v: &ClipVertex, bw: usize, bh: usize) -> ScreenVertex {
    let inv_w = 1.0 / v.p[3];
    let ndc_x = v.p[0] * inv_w;
    let ndc_y = v.p[1] * inv_w;
    ScreenVertex {
        x: (ndc_x * 0.5 + 0.5) * bw as f64,
        y: (1.0 - (ndc_y * 0.5 + 0.5)) * bh as f64,
        inv_w,
        n_over_w: scaled(v.n, inv_w),
    }
}

fn edge_fn(a: &ScreenVertex, b: &ScreenVertex, c: &ScreenVertex) -> f64 {
    (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)
}

/// Shade one front-facing sample: Lambert plus a hemisphere ambient, in linear
/// space, sRGB-encoded on the way out.
fn shade(normal: [f64; 3], albedo: [f64; 3], light: [f64; 3]) -> [u8; 3] {
    let n = normalize(normal);
    let sky = to_linear(HEMI_SKY);
    let ground = to_linear(HEMI_GROUND);
    let mix = 0.5 * dot(n, HEMI_UP) + 0.5;
    let ndl = dot(n, light).max(0.0) * LIGHT_INTENSITY;
    let mut out = [0.0; 3];
    for i in 0..3 {
        let ambient = (ground[i] + (sky[i] - ground[i]) * mix) * HEMI_INTENSITY;
        out[i] = albedo[i] * (ambient + ndl);
    }
    encode_srgb(out)
}

fn raster_triangle(
    fb: &mut FrameBuffer,
    v: [ScreenVertex; 3],
    front: bool,
    albedo: [f64; 3],
    back: [u8; 3],
    light: [f64; 3],
) {
    let area = edge_fn(&v[0], &v[1], &v[2]);
    if area == 0.0 || !area.is_finite() {
        return;
    }
    let x0 = v.iter().map(|p| p.x).fold(f64::INFINITY, f64::min).floor().max(0.0) as usize;
    let x1 = (v.iter().map(|p| p.x).fold(f64::NEG_INFINITY, f64::max).ceil()).min(fb.w as f64) as usize;
    let y0 = v.iter().map(|p| p.y).fold(f64::INFINITY, f64::min).floor().max(0.0) as usize;
    let y1 = (v.iter().map(|p| p.y).fold(f64::NEG_INFINITY, f64::max).ceil()).min(fb.h as f64) as usize;
    if x0 >= x1 || y0 >= y1 {
        return;
    }
    let inv_area = 1.0 / area;
    for py in y0..y1 {
        let sy = py as f64 + 0.5;
        for px in x0..x1 {
            let sx = px as f64 + 0.5;
            let p = ScreenVertex { x: sx, y: sy, inv_w: 0.0, n_over_w: [0.0; 3] };
            let w0 = edge_fn(&v[1], &v[2], &p) * inv_area;
            let w1 = edge_fn(&v[2], &v[0], &p) * inv_area;
            let w2 = edge_fn(&v[0], &v[1], &p) * inv_area;
            if w0 < 0.0 || w1 < 0.0 || w2 < 0.0 {
                continue;
            }
            // 1/w is linear in screen space, so depth and the attributes below
            // interpolate correctly with the same weights.
            let inv_w = w0 * v[0].inv_w + w1 * v[1].inv_w + w2 * v[2].inv_w;
            let i = py * fb.w + px;
            if (inv_w as f32) <= fb.depth[i] {
                continue;
            }
            fb.depth[i] = inv_w as f32;
            fb.color[i] = if front {
                let mut n = [0.0; 3];
                for k in 0..3 {
                    n[k] = (w0 * v[0].n_over_w[k] + w1 * v[1].n_over_w[k] + w2 * v[2].n_over_w[k]) / inv_w;
                }
                shade(n, albedo, light)
            } else {
                back
            };
        }
    }
}

// ---------------------------------------------------------------------------
// Feature edges — EdgesGeometry(geo, 30) drawn as 2 px screen-space lines
// ---------------------------------------------------------------------------

/// The edges `EdgesGeometry(geometry, thresholdAngle)` keeps: an edge shared by
/// two faces whose normals differ by more than the threshold, plus every
/// unmatched (boundary) edge. Vertices are identified by their position rounded
/// to four decimals, exactly as three.js hashes them.
fn feature_edges(verts: &[[f64; 3]], tris: &[[u32; 3]]) -> Vec<(u32, u32)> {
    let key = |p: [f64; 3]| -> [i64; 3] {
        [
            (p[0] * EDGE_HASH_PRECISION).round() as i64,
            (p[1] * EDGE_HASH_PRECISION).round() as i64,
            (p[2] * EDGE_HASH_PRECISION).round() as i64,
        ]
    };
    let threshold_dot = EDGE_ANGLE_DEG.to_radians().cos();
    // Half-edge -> (its two endpoints, the face normal), or None once matched.
    let mut pending: HashMap<([i64; 3], [i64; 3]), Option<(u32, u32, [f64; 3])>> = HashMap::new();
    let mut out: Vec<(u32, u32)> = Vec::new();
    for t in tris {
        let (a, b, c) = (verts[t[0] as usize], verts[t[1] as usize], verts[t[2] as usize]);
        let n = normalize(cross(sub(c, b), sub(a, b)));
        let keys = [key(a), key(b), key(c)];
        for j in 0..3 {
            let jn = (j + 1) % 3;
            let forward = (keys[j], keys[jn]);
            let reverse = (keys[jn], keys[j]);
            match pending.get_mut(&reverse) {
                Some(slot @ Some(_)) => {
                    let (i0, i1, other) = slot.take().unwrap();
                    if dot(n, other) <= threshold_dot {
                        out.push((i0, i1));
                    }
                }
                _ => {
                    pending.entry(forward).or_insert(Some((t[j], t[jn], n)));
                }
            }
        }
    }
    // Anything still unmatched is a boundary edge; three.js keeps those.
    for slot in pending.into_values().flatten() {
        out.push((slot.0, slot.1));
    }
    out
}

/// Draw the feature edges as depth-tested screen-space lines of
/// [`EDGE_WIDTH_PX`] output pixels, the way `LineMaterial({ linewidth: 2 })`
/// expands each segment into a quad.
fn draw_feature_edges(
    fb: &mut FrameBuffer,
    verts: &[[f64; 3]],
    tris: &[[u32; 3]],
    view_proj: &Mat4,
    color: [u8; 3],
    s: usize,
) {
    let half = EDGE_WIDTH_PX * s as f64 / 2.0;
    for (ia, ib) in feature_edges(verts, tris) {
        let ca = ClipVertex { p: transform_point(view_proj, verts[ia as usize]), n: [0.0; 3] };
        let cb = ClipVertex { p: transform_point(view_proj, verts[ib as usize]), n: [0.0; 3] };
        let poly = clip_near(&[ca, cb]);
        if poly.len() < 2 {
            continue;
        }
        let a = to_screen(&poly[0], fb.w, fb.h);
        let b = to_screen(&poly[1], fb.w, fb.h);
        draw_thick_segment(fb, a, b, half, color);
    }
}

fn draw_thick_segment(fb: &mut FrameBuffer, a: ScreenVertex, b: ScreenVertex, half: f64, color: [u8; 3]) {
    let (dx, dy) = (b.x - a.x, b.y - a.y);
    let len2 = dx * dx + dy * dy;
    let len = len2.sqrt();
    if !len.is_finite() {
        return;
    }
    // The quad's end caps extend half a width past each endpoint, as the line
    // shader's `offset += dir` does.
    let x0 = (a.x.min(b.x) - half).floor().max(0.0) as usize;
    let x1 = ((a.x.max(b.x) + half).ceil()).min(fb.w as f64).max(0.0) as usize;
    let y0 = (a.y.min(b.y) - half).floor().max(0.0) as usize;
    let y1 = ((a.y.max(b.y) + half).ceil()).min(fb.h as f64).max(0.0) as usize;
    if x0 >= x1 || y0 >= y1 {
        return;
    }
    for py in y0..y1 {
        let sy = py as f64 + 0.5;
        for px in x0..x1 {
            let sx = px as f64 + 0.5;
            // Distance to the segment, and the parameter used for its depth.
            let t = if len2 > 0.0 { (((sx - a.x) * dx + (sy - a.y) * dy) / len2).clamp(0.0, 1.0) } else { 0.0 };
            let (cx, cy) = (a.x + dx * t, a.y + dy * t);
            let along = if len > 0.0 { ((sx - a.x) * dx + (sy - a.y) * dy) / len } else { 0.0 };
            // Square-ish caps: inside the half width of the segment, or of the
            // half-width extension past either end.
            let d2 = (sx - cx) * (sx - cx) + (sy - cy) * (sy - cy);
            if d2 > half * half || along < -half || along > len + half {
                continue;
            }
            let inv_w = a.inv_w + (b.inv_w - a.inv_w) * t;
            let i = py * fb.w + px;
            // A feature edge lies exactly on the surface it belongs to, so it
            // needs a depth slack to win; without one it would z-fight away.
            if fb.depth[i] > f32::NEG_INFINITY && (inv_w * (1.0 + EDGE_DEPTH_SLACK)) as f32 <= fb.depth[i] {
                continue;
            }
            fb.depth[i] = inv_w as f32;
            fb.color[i] = color;
        }
    }
}

// ---------------------------------------------------------------------------
// Command line
// ---------------------------------------------------------------------------

/// What `render_mesh`'s command line accepts, printed on a usage error.
pub const CLI_USAGE: &str = "\
render-mesh <mesh.json> <out.png> [options]
render-mesh --compare <a.json> <b.json> [--out-a <png>] [--out-b <png>] [options]
render-mesh --compare-png <a.png> <b.png>

Framing (render.mjs's three modes; the default auto-frames each mesh):
  --normalize                 recentre and rescale onto the unit sphere, so two
                              meshes of the same shape land on identical pixels
                              (--compare always does this, as compareSilhouettes did)
  --frame cx,cy,cz,r          an explicit camera centre and radius

Options:
  --size WxH                  output size (default 960x720)
  --samples N                 supersampling per axis, 1..8 (default 4)
  --color #rrggbb             front-face colour
  --edge-color #rrggbb        feature-edge colour
  --color-a/--color-b         per-mesh front colours under --compare
  --edge-color-a/--edge-color-b   per-mesh edge colours under --compare
  --no-edges                  skip the feature-edge pass
  --json                      also print the pixel counts as JSON

Every mode prints its measurements as JSON on stdout: opaque and red (back-face)
pixel counts per mesh, and for the comparison modes the silhouette intersection,
union, IoU, mismatch and each mesh's inside-out fraction.";

fn read_mesh(path: &str) -> Result<Mesh, String> {
    let text = std::fs::read_to_string(path).map_err(|e| format!("{path}: {e}"))?;
    Mesh::from_json_str(&text)
}

fn read_image(path: &str) -> Result<RgbaImage, String> {
    let bytes = std::fs::read(path).map_err(|e| format!("{path}: {e}"))?;
    crate::image::decode_png(&bytes).map_err(|e| format!("{path}: {e}"))
}

fn write_png(render: &Render, path: &str) -> Result<(), String> {
    std::fs::write(path, render.png()?).map_err(|e| format!("{path}: {e}"))
}

fn comparison_json(c: &Comparison) -> serde_json::Value {
    serde_json::json!({
        "opaqueA": c.opaque_a,
        "opaqueB": c.opaque_b,
        "redA": c.red_a,
        "redB": c.red_b,
        "intersection": c.intersection,
        "union": c.union,
        "iou": c.iou,
        "mismatch": c.mismatch,
        "backfaceA": c.backface_a,
        "backfaceB": c.backface_b,
    })
}

fn silhouette_json(s: &Silhouette) -> serde_json::Value {
    serde_json::json!({
        "width": s.width,
        "height": s.height,
        "opaque": s.opaque,
        "red": s.red,
        "backface": s.backface_fraction(),
    })
}

/// Run the `render-mesh` command line and return what it should print.
///
/// `args` excludes the program name. Shared by the `render_mesh` example binary
/// and `brep-mcp render-mesh`.
pub fn cli(args: &[String]) -> Result<String, String> {
    let mut opts = RenderOptions::default();
    let mut positional: Vec<String> = Vec::new();
    let mut mode_compare = false;
    let mut mode_compare_png = false;
    let mut out_a: Option<String> = None;
    let mut out_b: Option<String> = None;
    let mut color_a: Option<[u8; 3]> = None;
    let mut color_b: Option<[u8; 3]> = None;
    let mut edge_a: Option<[u8; 3]> = None;
    let mut edge_b: Option<[u8; 3]> = None;
    let mut json = false;

    let mut i = 0;
    let need = |i: usize, flag: &str| -> Result<String, String> {
        args.get(i + 1).cloned().ok_or_else(|| format!("{flag} needs a value"))
    };
    while i < args.len() {
        let a = args[i].as_str();
        match a {
            "--help" | "-h" => return Ok(CLI_USAGE.to_string()),
            "--compare" => mode_compare = true,
            "--compare-png" => mode_compare_png = true,
            "--normalize" => opts.framing = Framing::Normalize,
            "--no-edges" => opts.edges = false,
            "--json" => json = true,
            "--frame" => {
                let v = need(i, a)?;
                let parts: Vec<f64> = v
                    .split(',')
                    .map(|p| p.trim().parse::<f64>().map_err(|e| format!("--frame {v:?}: {e}")))
                    .collect::<Result<_, _>>()?;
                if parts.len() != 4 {
                    return Err(format!("--frame wants cx,cy,cz,r, got {v:?}"));
                }
                opts.framing = Framing::Explicit { center: [parts[0], parts[1], parts[2]], radius: parts[3] };
                i += 1;
            }
            "--size" => {
                let v = need(i, a)?;
                let (w, h) = v.split_once(['x', 'X']).ok_or_else(|| format!("--size wants WxH, got {v:?}"))?;
                opts.width = w.trim().parse().map_err(|e| format!("--size {v:?}: {e}"))?;
                opts.height = h.trim().parse().map_err(|e| format!("--size {v:?}: {e}"))?;
                i += 1;
            }
            "--samples" => {
                opts.samples = need(i, a)?.parse().map_err(|e| format!("--samples: {e}"))?;
                i += 1;
            }
            "--color" => {
                opts.color = parse_color(&need(i, a)?)?;
                i += 1;
            }
            "--edge-color" => {
                opts.edge_color = parse_color(&need(i, a)?)?;
                i += 1;
            }
            "--color-a" => {
                color_a = Some(parse_color(&need(i, a)?)?);
                i += 1;
            }
            "--color-b" => {
                color_b = Some(parse_color(&need(i, a)?)?);
                i += 1;
            }
            "--edge-color-a" => {
                edge_a = Some(parse_color(&need(i, a)?)?);
                i += 1;
            }
            "--edge-color-b" => {
                edge_b = Some(parse_color(&need(i, a)?)?);
                i += 1;
            }
            "--out-a" => {
                out_a = Some(need(i, a)?);
                i += 1;
            }
            "--out-b" => {
                out_b = Some(need(i, a)?);
                i += 1;
            }
            _ if a.starts_with("--") => return Err(format!("unknown option {a}\n\n{CLI_USAGE}")),
            _ => positional.push(a.to_string()),
        }
        i += 1;
    }

    if mode_compare_png {
        if positional.len() != 2 {
            return Err(format!("--compare-png wants two PNG paths\n\n{CLI_USAGE}"));
        }
        let a = silhouette_of(&read_image(&positional[0])?);
        let b = silhouette_of(&read_image(&positional[1])?);
        return Ok(serde_json::to_string_pretty(&comparison_json(&compare(&a, &b))).unwrap());
    }

    if mode_compare {
        if positional.len() != 2 {
            return Err(format!("--compare wants two mesh JSON paths\n\n{CLI_USAGE}"));
        }
        let mesh_a = read_mesh(&positional[0])?;
        let mesh_b = read_mesh(&positional[1])?;
        let mut oa = opts.clone();
        let mut ob = opts.clone();
        if let Some(c) = color_a {
            oa.color = c;
        }
        if let Some(c) = color_b {
            ob.color = c;
        }
        if let Some(c) = edge_a {
            oa.edge_color = c;
        }
        if let Some(c) = edge_b {
            ob.edge_color = c;
        }
        let (ra, rb, cmp) = compare_meshes(&mesh_a, &mesh_b, &oa, &ob)?;
        if let Some(p) = &out_a {
            write_png(&ra, p)?;
        }
        if let Some(p) = &out_b {
            write_png(&rb, p)?;
        }
        return Ok(serde_json::to_string_pretty(&comparison_json(&cmp)).unwrap());
    }

    if positional.len() != 2 {
        return Err(format!("expected <mesh.json> <out.png>\n\n{CLI_USAGE}"));
    }
    let mesh = read_mesh(&positional[0])?;
    let r = render(&mesh, &opts)?;
    write_png(&r, &positional[1])?;
    Ok(if json {
        serde_json::to_string_pretty(&silhouette_json(&r.silhouette)).unwrap()
    } else {
        String::new()
    })
}

