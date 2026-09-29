// brep-render core shaders: shaded faces + screen-space-width edge overlay +
// screen-constant vertex points, all style-driven (R14 material variants).
//
// Face lighting is a linear-space port of the retired artifact page's rig
// (hemisphere 0xbfd4ff/0x202028 @0.9 + key directional @1.4 + fill @0.45,
// Phong shininess 24), view-stable (R15): light directions live in world space
// but the look was tuned against the same rig the retired viewer used. The render target is
// NON-sRGB; fragment shaders encode to sRGB explicitly so the background clear
// (sRGB bytes) needs no conversion.

struct Globals {
    view_proj: mat4x4<f32>,
    // Viewport size in physical pixels (xy), device pixel ratio (z); w unused.
    viewport: vec4<f32>,
    // World-space view direction, camera -> scene (xyz); w unused.
    forward: vec4<f32>,
};

// One style = one material variant (base/selected/hover per kind).
//   faces:  color in LINEAR space (converted f64-exactly at upload, keeping
//           artifact bytes stable) + params.x = flat-shading flag (>0.5)
//   edges:  color in sRGB (written to the non-sRGB target as-is) + alpha;
//           params.x = width in CSS px, params.y = depth nudge
//   points: color in sRGB + alpha; params.x = diameter in CSS px
struct Style {
    color: vec4<f32>,
    params: vec4<f32>,
};

@group(0) @binding(0) var<uniform> globals: Globals;
@group(1) @binding(0) var<uniform> style: Style;

fn srgb_encode(c: vec3<f32>) -> vec3<f32> {
    let lo = c * 12.92;
    let hi = 1.055 * pow(max(c, vec3<f32>(0.0)), vec3<f32>(1.0 / 2.4)) - 0.055;
    return select(hi, lo, c <= vec3<f32>(0.0031308));
}

// ---------------------------------------------------------------------------
// Shaded faces
// ---------------------------------------------------------------------------

struct MeshOut {
    @builtin(position) position: vec4<f32>,
    @location(0) normal: vec3<f32>,
    @location(1) world: vec3<f32>,
};

@vertex
fn vs_mesh(
    @location(0) position: vec3<f32>,
    @location(1) normal: vec3<f32>,
) -> MeshOut {
    var out: MeshOut;
    out.position = globals.view_proj * vec4<f32>(position, 1.0);
    out.normal = normal;
    out.world = position;
    return out;
}

@fragment
fn fs_mesh(in: MeshOut) -> @location(0) vec4<f32> {
    var n = normalize(in.normal);
    if (style.params.x > 0.5) {
        // Flat shading: face normal from screen-space derivatives.
        n = normalize(cross(dpdx(in.world), dpdy(in.world)));
    }
    // Double-sided: flip the normal toward the camera.
    let v = -globals.forward.xyz;
    if (dot(n, v) < 0.0) {
        n = -n;
    }

    // Hemisphere light (up +Y): sky 0xbfd4ff, ground 0x202028,
    // intensity 0.9 (colors pre-converted to linear here).
    let sky = vec3<f32>(0.5209, 0.6584, 1.0);
    let ground = vec3<f32>(0.0144, 0.0144, 0.0212);
    let hemi = mix(ground, sky, n.y * 0.5 + 0.5) * 0.9;

    // Key + fill directionals (white).
    let l_key = normalize(vec3<f32>(1.0, -1.2, 1.5));
    let l_fill = normalize(vec3<f32>(-0.6, 0.7, 0.3));
    let diffuse_light = hemi
        + vec3<f32>(1.4) * max(dot(n, l_key), 0.0)
        + vec3<f32>(0.45) * max(dot(n, l_fill), 0.0);

    // Phong specular, key light only (MeshPhongMaterial specular 0x111111,
    // shininess 24; 0x11/255 -> linear ~0.0056).
    let refl = reflect(-l_key, n);
    let spec = vec3<f32>(0.0056) * pow(max(dot(refl, v), 0.0), 24.0) * 1.4;

    let color = style.color.rgb * diffuse_light + spec;
    return vec4<f32>(srgb_encode(color), 1.0);
}

// Wireframe: the tessellated triangle mesh drawn as lines. Flat base color (no
// lighting), lifted a touch so the mesh reads clearly against the background.
@fragment
fn fs_wire(in: MeshOut) -> @location(0) vec4<f32> {
    return vec4<f32>(srgb_encode(style.color.rgb * 1.15 + vec3<f32>(0.03)), 1.0);
}

// ---------------------------------------------------------------------------
// Edge overlay: screen-constant-width lines (R16). Each SEGMENT is one
// instance (p0, p1); six vertices expand it to a screen-aligned quad, with the
// endpoints extended by the half-width for join coverage.
// ---------------------------------------------------------------------------

struct EdgeOut {
    @builtin(position) position: vec4<f32>,
};

@vertex
fn vs_edge(
    @builtin(vertex_index) vertex_index: u32,
    @location(0) p0: vec3<f32>,
    @location(1) p1: vec3<f32>,
) -> EdgeOut {
    // (end, side) per corner; two CCW triangles of the quad.
    var corners = array<vec2<f32>, 6>(
        vec2<f32>(0.0, -1.0), vec2<f32>(1.0, -1.0), vec2<f32>(0.0, 1.0),
        vec2<f32>(0.0, 1.0), vec2<f32>(1.0, -1.0), vec2<f32>(1.0, 1.0),
    );
    let corner = corners[vertex_index];

    let clip0 = globals.view_proj * vec4<f32>(p0, 1.0);
    let clip1 = globals.view_proj * vec4<f32>(p1, 1.0);
    let half_vp = globals.viewport.xy * 0.5;
    let px0 = clip0.xy / clip0.w * half_vp;
    let px1 = clip1.xy / clip1.w * half_vp;
    var dir = px1 - px0;
    let len = length(dir);
    if (len < 1e-6) {
        dir = vec2<f32>(1.0, 0.0);
    } else {
        dir = dir / len;
    }
    let perp = vec2<f32>(-dir.y, dir.x);
    // Width is in CSS px; scale by the DPR into physical pixels.
    let half_w = style.params.x * max(globals.viewport.z, 1e-3) * 0.5;

    var clip = clip0;
    var px = px0;
    var along = -dir; // extend the start cap backward
    if (corner.x > 0.5) {
        clip = clip1;
        px = px1;
        along = dir;
    }
    let offset_px = perp * (corner.y * half_w) + along * half_w;
    let ndc_offset = offset_px / half_vp * clip.w;

    // NDC depth nudge toward the camera so boundary edges win the z-fight with
    // their own faces (faces also carry a depth bias pushing them back).
    let nudge = style.params.y;
    var out: EdgeOut;
    out.position = vec4<f32>(clip.xy + ndc_offset, clip.z - nudge * clip.w, clip.w);
    return out;
}

@fragment
fn fs_edge() -> @location(0) vec4<f32> {
    return style.color;
}

// ---------------------------------------------------------------------------
// Vertex points: screen-constant-size round sprites (R16), one instance per
// topology vertex.
// ---------------------------------------------------------------------------

struct PointOut {
    @builtin(position) position: vec4<f32>,
    @location(0) local: vec2<f32>,
};

@vertex
fn vs_point(
    @builtin(vertex_index) vertex_index: u32,
    @location(0) center: vec3<f32>,
) -> PointOut {
    var corners = array<vec2<f32>, 6>(
        vec2<f32>(-1.0, -1.0), vec2<f32>(1.0, -1.0), vec2<f32>(-1.0, 1.0),
        vec2<f32>(-1.0, 1.0), vec2<f32>(1.0, -1.0), vec2<f32>(1.0, 1.0),
    );
    let corner = corners[vertex_index];
    let clip = globals.view_proj * vec4<f32>(center, 1.0);
    let half_vp = globals.viewport.xy * 0.5;
    let radius_px = style.params.x * max(globals.viewport.z, 1e-3) * 0.5;
    let ndc_offset = corner * radius_px / half_vp * clip.w;

    var out: PointOut;
    // Nudge points toward the camera so they sit on top of edges/faces.
    out.position = vec4<f32>(clip.xy + ndc_offset, clip.z - 3e-4 * clip.w, clip.w);
    out.local = corner;
    return out;
}

@fragment
fn fs_point(in: PointOut) -> @location(0) vec4<f32> {
    let r = length(in.local);
    // Round sprite with a ~1px anti-aliased rim.
    let radius_px = style.params.x * max(globals.viewport.z, 1e-3) * 0.5;
    let aa = clamp(1.0 - (r - 1.0) * radius_px, 0.0, 1.0);
    if (aa <= 0.0) {
        discard;
    }
    return vec4<f32>(style.color.rgb, style.color.a * aa);
}

// ---------------------------------------------------------------------------
// Overlay widgets: the brep-gizmos `Overlay` — per-vertex-colored
// triangles + per-instance-colored screen-constant-width lines — drawn over the
// solids in their own depth-cleared pass (transform gizmo, ViewCube, datum /
// dimension / curve visuals). Colors are treated as display (sRGB) values and
// written to the non-sRGB target directly (matching the edge overlay), with a
// cheap sRGB-space shade on triangles so faces read with depth. `style.params.x`
// carries the overlay line width in CSS px (fed per pass).
// ---------------------------------------------------------------------------

struct OverlayTriOut {
    @builtin(position) position: vec4<f32>,
    @location(0) normal: vec3<f32>,
    @location(1) color: vec4<f32>,
};

@vertex
fn vs_overlay_tri(
    @location(0) position: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) color: vec4<f32>,
) -> OverlayTriOut {
    var out: OverlayTriOut;
    out.position = globals.view_proj * vec4<f32>(position, 1.0);
    out.normal = normal;
    out.color = color;
    return out;
}

@fragment
fn fs_overlay_tri(in: OverlayTriOut) -> @location(0) vec4<f32> {
    // Soft two-sided sRGB-space shade so widget faces read without swimming;
    // preserves the vertex color at full light so lines and tris match.
    var n = normalize(in.normal);
    let v = -globals.forward.xyz;
    if (dot(n, v) < 0.0) {
        n = -n;
    }
    let key = normalize(vec3<f32>(0.4, -0.6, 0.8));
    let shade = 0.62 + 0.38 * max(dot(n, key), 0.0);
    return vec4<f32>(in.color.rgb * shade, in.color.a);
}

struct OverlayLineOut {
    @builtin(position) position: vec4<f32>,
    @location(0) color: vec4<f32>,
};

@vertex
fn vs_overlay_line(
    @builtin(vertex_index) vertex_index: u32,
    @location(0) p0: vec3<f32>,
    @location(1) p1: vec3<f32>,
    @location(2) color: vec4<f32>,
) -> OverlayLineOut {
    var corners = array<vec2<f32>, 6>(
        vec2<f32>(0.0, -1.0), vec2<f32>(1.0, -1.0), vec2<f32>(0.0, 1.0),
        vec2<f32>(0.0, 1.0), vec2<f32>(1.0, -1.0), vec2<f32>(1.0, 1.0),
    );
    let corner = corners[vertex_index];

    let clip0 = globals.view_proj * vec4<f32>(p0, 1.0);
    let clip1 = globals.view_proj * vec4<f32>(p1, 1.0);
    let half_vp = globals.viewport.xy * 0.5;
    let px0 = clip0.xy / clip0.w * half_vp;
    let px1 = clip1.xy / clip1.w * half_vp;
    var dir = px1 - px0;
    let len = length(dir);
    if (len < 1e-6) {
        dir = vec2<f32>(1.0, 0.0);
    } else {
        dir = dir / len;
    }
    let perp = vec2<f32>(-dir.y, dir.x);
    let half_w = style.params.x * max(globals.viewport.z, 1e-3) * 0.5;

    var clip = clip0;
    var along = -dir;
    if (corner.x > 0.5) {
        clip = clip1;
        along = dir;
    }
    let offset_px = perp * (corner.y * half_w) + along * half_w;
    let ndc_offset = offset_px / half_vp * clip.w;

    // Small nudge toward the camera so borders read on top of their own fills.
    var out: OverlayLineOut;
    out.position = vec4<f32>(clip.xy + ndc_offset, clip.z - 1.0e-4 * clip.w, clip.w);
    out.color = color;
    return out;
}

@fragment
fn fs_overlay_line(in: OverlayLineOut) -> @location(0) vec4<f32> {
    return in.color;
}
