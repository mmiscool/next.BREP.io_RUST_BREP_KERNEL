//! The wgpu render core — windowing-agnostic (the dual-target seam).
//!
//! [`RenderCore::render_to_view`] draws a [`RenderScene`] into ANY
//! `wgpu::TextureView` (MSAA 4x): shaded faces with per-face selection/hover
//! emphasis (R17), screen-constant-width edges with occluded portions dimmed
//! rather than dropped (R17), selected-face boundary outlines, vertex point
//! sprites, and the world-axis helper (R20). Presentation shells stay thin:
//! - headless: [`RenderCore::render_to_png`] (render-to-texture → readback →
//!   PNG bytes) — the artifact binary and R34 screenshot capture;
//! - desktop: the winit shell hands the surface texture's view here;
//! - web: the wasm canvas shell does the same.
//!
//! GPU buffers are retained per solid and reused while the solid's scene
//! revision is unchanged (R10 — reused solids keep their buffers across
//! history reruns). Determinism (R33): same scene + camera + size + styles ⇒
//! the same command stream on the same device ⇒ identical PNG bytes.

use crate::camera::Camera;
use crate::color::{solid_color_srgb, srgb_to_linear};
use crate::scene::{RenderScene, SolidDisplay};
use crate::style::{Emphasis, EmphasisState, FaceColorMode, RenderSettings, Rgba};
use std::collections::{HashMap, HashSet};
use wgpu::util::DeviceExt;

/// MSAA sample count (R7-equivalent quality).
pub const SAMPLES: u32 = 4;
/// Offscreen color format. Non-sRGB: the shaders encode explicitly, so the
/// clear color's bytes land in the PNG exactly.
pub const COLOR_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Rgba8Unorm;
const DEPTH_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Depth32Float;

/// NDC depth nudge for boundary edges (they must win the z-fight with their
/// own faces; faces also carry a rasterizer depth bias pushing them back).
const EDGE_NUDGE: f32 = 2e-4;

#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct Globals {
    view_proj: [[f32; 4]; 4],
    viewport: [f32; 4],
    forward: [f32; 4],
}

#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct StyleParams {
    color: [f32; 4],
    params: [f32; 4],
}

#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct MeshVertex {
    position: [f32; 3],
    normal: [f32; 3],
}

#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct EdgeInstance {
    p0: [f32; 3],
    p1: [f32; 3],
}

#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct PointInstance {
    center: [f32; 3],
}

/// One overlay-widget line segment (per-instance colored, screen-constant width).
#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct OverlayLineInstance {
    p0: [f32; 3],
    p1: [f32; 3],
    color: [f32; 4],
}

/// One overlay-widget triangle vertex (per-vertex colored, shaded).
#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct OverlayTriVertex {
    position: [f32; 3],
    normal: [f32; 3],
    color: [f32; 4],
}

/// One uniform style buffer + its bind group (a material variant).
struct StyleBuf {
    buf: wgpu::Buffer,
    bind: wgpu::BindGroup,
}

impl StyleBuf {
    fn new(device: &wgpu::Device, layout: &wgpu::BindGroupLayout, label: &str) -> Self {
        let buf = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some(label),
            size: std::mem::size_of::<StyleParams>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let bind = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some(label),
            layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: buf.as_entire_binding(),
            }],
        });
        Self { buf, bind }
    }

    fn write(&self, queue: &wgpu::Queue, color: Rgba, params: [f32; 4]) {
        queue.write_buffer(&self.buf, 0, bytemuck::bytes_of(&StyleParams { color, params }));
    }
}

/// The global material variants (updated from [`RenderSettings`] each frame).
struct GlobalStyles {
    face_selected: StyleBuf,
    face_hovered: StyleBuf,
    edge_base: StyleBuf,
    edge_selected: StyleBuf,
    edge_hovered: StyleBuf,
    edge_hidden: StyleBuf,
    /// A display-only (AUX) edge at rest — a board's copper outline.
    edge_aux: StyleBuf,
    boundary: StyleBuf,
    point_base: StyleBuf,
    point_selected: StyleBuf,
    point_hovered: StyleBuf,
    axis_x: StyleBuf,
    axis_y: StyleBuf,
    axis_z: StyleBuf,
    /// Overlay-widget line width (CSS px in `params.x`); color is per-instance.
    overlay_line: StyleBuf,
}

struct FaceRange {
    first_index: u32,
    index_count: u32,
    /// This face's slice of the WIREFRAME index buffer. Not derivable from
    /// `first_index`: the wire buffer holds one segment per DISTINCT mesh edge
    /// of the face, so its length depends on how the face's triangles share
    /// edges, not on how many there are.
    wire_first: u32,
    wire_count: u32,
    /// Index into [`GpuSolid::face_styles`] when this face carries its OWN
    /// colour (a `color` metadata attribute on the face name); `None` = shade
    /// with the solid's `base_style`.
    style: Option<u32>,
}

/// The hasher behind the wireframe's per-face edge de-duplication.
///
/// The keys are pairs of mesh vertex INDICES — dense, small, and entirely under
/// our control, so there is no adversary for SipHash to defend against, and its
/// per-key cost is the whole cost of the de-duplication (three probes per
/// triangle, at every solid upload). This is the multiply-xor mix `rustc-hash`
/// uses, written out rather than depended on: `rustc-hash` only rides into this
/// build through wgpu's internals, and a published crate should not take a
/// direct dependency for fifteen lines.
#[derive(Default)]
struct EdgeHasher(u64);

impl std::hash::Hasher for EdgeHasher {
    fn write(&mut self, bytes: &[u8]) {
        for &byte in bytes {
            self.write_u32(u32::from(byte));
        }
    }

    fn write_u32(&mut self, value: u32) {
        // 2^64 / golden ratio: the odd multiplier spreads low-index bits into
        // the high ones, which is where `finish` folds them back down from.
        self.0 = (self.0 ^ u64::from(value)).wrapping_mul(0x9E37_79B9_7F4A_7C15);
    }

    fn finish(&self) -> u64 {
        self.0 ^ (self.0 >> 32)
    }
}

type BuildEdgeHasher = std::hash::BuildHasherDefault<EdgeHasher>;

struct EdgeRange {
    first_instance: u32,
    instance_count: u32,
}

struct GpuSolid {
    revision: u64,
    vertex_buf: wgpu::Buffer,
    index_buf: wgpu::Buffer,
    /// Line-list indices for the wireframe view (each triangle → its 3 edges).
    wire_index_buf: wgpu::Buffer,
    wire_index_count: u32,
    faces: Vec<FaceRange>,
    edge_buf: Option<wgpu::Buffer>,
    edges: Vec<EdgeRange>,
    edge_instances: u32,
    /// The instances drawn by the OCCLUDED-edge pass: every edge's up to the
    /// first AUX one. A display-only outline (a board's copper) is not dimmed
    /// through what hides it — the bottom copper would ghost through the board
    /// — and aux edges trail the real ones, so it is a prefix.
    occludable_edge_instances: u32,
    point_buf: Option<wgpu::Buffer>,
    point_count: u32,
    /// Per-solid base face style (uniform or hashed color; metadata override).
    base_style: StyleBuf,
    /// De-duplicated palette of PER-FACE styles — one entry per distinct face
    /// colour on this solid, referenced by [`FaceRange::style`]. Empty for the
    /// overwhelmingly common solid whose faces are all one colour, which is what
    /// keeps the whole-mesh fast path in `frame.rs` alive.
    face_styles: Vec<StyleBuf>,
    /// Selected/hovered face boundary outline segments (cache keyed by the
    /// emphasis generation).
    boundary: Option<BoundaryBuf>,
}

struct BoundaryBuf {
    emphasis_generation: u64,
    revision: u64,
    buf: Option<wgpu::Buffer>,
    count: u32,
}

/// A scene uploaded to GPU buffers, retained across frames; sync with
/// [`RenderCore::sync_scene`].
#[derive(Default)]
pub struct GpuScene {
    solids: HashMap<String, GpuSolid>,
    /// Draw order (scene insertion order at last sync).
    order: Vec<String>,
    settings_generation: u64,
    axis_buf: Option<wgpu::Buffer>,
    /// Count of solid (re)uploads (R10 observability: a reused solid must NOT
    /// bump this across a sync).
    uploads: u64,
}

impl GpuScene {
    /// Total solid buffer uploads since creation (test/telemetry hook for the
    /// reused-buffer fast path).
    pub fn upload_count(&self) -> u64 {
        self.uploads
    }
}

struct CachedTargets {
    width: u32,
    height: u32,
    msaa_view: wgpu::TextureView,
    depth_view: wgpu::TextureView,
}

/// Everything a frame needs beyond the scene buffers.
pub struct FrameParams<'a> {
    pub camera: &'a Camera,
    /// Physical pixel size of the target.
    pub width: u32,
    pub height: u32,
    /// Device pixel ratio (CSS px → physical px) for line/point widths.
    pub dpr: f32,
    pub settings: &'a RenderSettings,
    pub emphasis: &'a Emphasis,
    /// World units per CSS pixel (for the screen-sized axis helper).
    pub world_per_pixel: f64,
    /// In-scene overlay widgets: transform gizmo, ViewCube, datum /
    /// dimension / curve visuals. `None` on the artifact/headless path.
    pub overlay: Option<&'a crate::widgets::WidgetOverlay>,
}

/// Create a headless device: adapter picked by `BREP_RENDER_ADAPTER`
/// (case-insensitive substring of the adapter name, e.g. "llvmpipe") or wgpu's
/// high-performance default. Native only — the browser shell gets its device
/// from the canvas context.
#[cfg(not(target_arch = "wasm32"))]
pub fn create_headless_device() -> Result<(wgpu::Device, wgpu::Queue, String), String> {
    let instance = wgpu::Instance::new(wgpu::InstanceDescriptor::new_without_display_handle_from_env());
    let want = std::env::var("BREP_RENDER_ADAPTER").ok();
    let adapter = if let Some(want) = &want {
        let want_lower = want.to_lowercase();
        pollster::block_on(instance.enumerate_adapters(wgpu::Backends::all()))
            .into_iter()
            .find(|adapter| adapter.get_info().name.to_lowercase().contains(&want_lower))
            .ok_or_else(|| format!("no adapter matching BREP_RENDER_ADAPTER={want}"))?
    } else {
        pollster::block_on(instance.request_adapter(&wgpu::RequestAdapterOptions {
            power_preference: wgpu::PowerPreference::HighPerformance,
            compatible_surface: None,
            force_fallback_adapter: false,
        }))
        .map_err(|error| format!("no wgpu adapter available: {error}"))?
    };
    let info = adapter.get_info();
    let (device, queue) = pollster::block_on(adapter.request_device(&wgpu::DeviceDescriptor {
        label: Some("brep-render"),
        ..Default::default()
    }))
    .map_err(|error| format!("wgpu device request failed: {error}"))?;
    Ok((device, queue, format!("{} ({:?})", info.name, info.backend)))
}

/// The window-agnostic renderer: pipelines + layouts for one color format.
pub struct RenderCore {
    pub device: wgpu::Device,
    pub queue: wgpu::Queue,
    format: wgpu::TextureFormat,
    globals_buf: wgpu::Buffer,
    globals_bind: wgpu::BindGroup,
    style_layout: wgpu::BindGroupLayout,
    mesh_pipeline: wgpu::RenderPipeline,
    wire_pipeline: wgpu::RenderPipeline,
    edge_visible_pipeline: wgpu::RenderPipeline,
    edge_hidden_pipeline: wgpu::RenderPipeline,
    point_pipeline: wgpu::RenderPipeline,
    /// Overlay-widget passes: shaded per-vertex tris + screen-constant
    /// per-instance lines, drawn over the solids in a depth-cleared pass.
    overlay_tri_pipeline: wgpu::RenderPipeline,
    /// Same as `overlay_tri_pipeline` but WITHOUT depth-write — for the datum/
    /// construction PLANES, which are translucent UI aids that must NOT occlude
    /// the gizmos/dimensions drawn after them (a plane no longer hides an offset
    /// arrow behind it). The gizmo tris still use the depth-writing pipeline so
    /// they self-occlude correctly.
    overlay_tri_nodepth_pipeline: wgpu::RenderPipeline,
    overlay_line_pipeline: wgpu::RenderPipeline,
    /// Second globals buffer for the ViewCube pass (its own mini-camera).
    vc_globals_buf: wgpu::Buffer,
    vc_globals_bind: wgpu::BindGroup,
    styles: GlobalStyles,
    targets: Option<CachedTargets>,
}

/// 8-bit RGB PNG encode (no text/time chunks — byte-deterministic).
#[cfg(not(target_arch = "wasm32"))]
fn encode_png(rgb: &[u8], width: u32, height: u32) -> Result<Vec<u8>, String> {
    let mut out = Vec::new();
    {
        let mut encoder = png::Encoder::new(&mut out, width, height);
        encoder.set_color(png::ColorType::Rgb);
        encoder.set_depth(png::BitDepth::Eight);
        let mut writer = encoder
            .write_header()
            .map_err(|error| format!("png header: {error}"))?;
        writer
            .write_image_data(rgb)
            .map_err(|error| format!("png data: {error}"))?;
    }
    Ok(out)
}

mod pipelines;
mod scene_sync;
mod frame;
