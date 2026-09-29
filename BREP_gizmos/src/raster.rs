//! A tiny CPU rasterizer for headless gizmo demos: projects an [`Overlay`]'s
//! triangles (z-buffered, flat-lit) and lines (screen-space width) through a
//! [`GizmoCamera`] and writes a PNG. NOT the production render path — the
//! engine draws overlays on the GPU — this exists only so a gizmo can be
//! eyeballed in isolation during development.

use crate::{GizmoCamera, Overlay, Vec3};

const BG: [u8; 3] = [16, 20, 28];

/// Render an overlay (optionally over a faint reference grid) to PNG bytes.
pub fn render_overlay_png(
    overlay: &Overlay,
    camera: &GizmoCamera,
    width: u32,
    height: u32,
) -> Vec<u8> {
    let w = width as usize;
    let h = height as usize;
    let mut rgb = vec![0u8; w * h * 3];
    for i in 0..w * h {
        rgb[i * 3] = BG[0];
        rgb[i * 3 + 1] = BG[1];
        rgb[i * 3 + 2] = BG[2];
    }
    let mut depth = vec![f32::INFINITY; w * h];

    let light = Vec3::new(-0.4, -0.6, 0.8).normalized();

    // Triangles: z-buffered, flat Lambert + ambient.
    for tri in overlay.tris.chunks_exact(3) {
        let sp: Vec<Option<[f32; 3]>> = tri
            .iter()
            .map(|v| {
                camera
                    .world_to_screen(Vec3::from(v.pos))
                    .map(|s| [s[0], s[1], view_depth(camera, Vec3::from(v.pos))])
            })
            .collect();
        if sp.iter().any(|s| s.is_none()) {
            continue;
        }
        let p: Vec<[f32; 3]> = sp.into_iter().map(|s| s.unwrap()).collect();
        let n = Vec3::from(tri[0].normal);
        let lambert = 0.35 + 0.65 * n.dot(light).abs();
        let base = tri[0].color;
        let col = [
            (base[0] * lambert).min(1.0),
            (base[1] * lambert).min(1.0),
            (base[2] * lambert).min(1.0),
        ];
        fill_triangle(&mut rgb, &mut depth, w, h, &p, col);
    }

    // Lines: 2px screen-space, depth-tested loosely (drawn after tris, biased
    // toward the viewer so gizmo lines read on top).
    for seg in overlay.lines.chunks_exact(2) {
        let a = camera.world_to_screen(Vec3::from(seg[0].pos));
        let b = camera.world_to_screen(Vec3::from(seg[1].pos));
        if let (Some(a), Some(b)) = (a, b) {
            let col = [seg[0].color[0], seg[0].color[1], seg[0].color[2]];
            draw_line(&mut rgb, w, h, a, b, col, 2);
        }
    }

    encode_png(&rgb, width, height)
}

fn view_depth(camera: &GizmoCamera, p: Vec3) -> f32 {
    // Distance along view direction from eye — a stable z key.
    p.sub(camera.eye).dot(camera.forward)
}

fn fill_triangle(
    rgb: &mut [u8],
    depth: &mut [f32],
    w: usize,
    h: usize,
    p: &[[f32; 3]],
    col: [f32; 3],
) {
    let min_x = p.iter().map(|v| v[0]).fold(f32::INFINITY, f32::min).floor().max(0.0) as usize;
    let max_x = p.iter().map(|v| v[0]).fold(f32::NEG_INFINITY, f32::max).ceil().min(w as f32 - 1.0) as usize;
    let min_y = p.iter().map(|v| v[1]).fold(f32::INFINITY, f32::min).floor().max(0.0) as usize;
    let max_y = p.iter().map(|v| v[1]).fold(f32::NEG_INFINITY, f32::max).ceil().min(h as f32 - 1.0) as usize;
    let (a, b, c) = (p[0], p[1], p[2]);
    let area = edge(a, b, c);
    if area.abs() < 1e-6 {
        return;
    }
    for y in min_y..=max_y.max(min_y) {
        for x in min_x..=max_x.max(min_x) {
            let px = [x as f32 + 0.5, y as f32 + 0.5, 0.0];
            let w0 = edge(b, c, px) / area;
            let w1 = edge(c, a, px) / area;
            let w2 = edge(a, b, px) / area;
            if w0 < -1e-4 || w1 < -1e-4 || w2 < -1e-4 {
                continue;
            }
            let z = w0 * a[2] + w1 * b[2] + w2 * c[2];
            let idx = y * w + x;
            if z < depth[idx] {
                depth[idx] = z;
                rgb[idx * 3] = (col[0] * 255.0) as u8;
                rgb[idx * 3 + 1] = (col[1] * 255.0) as u8;
                rgb[idx * 3 + 2] = (col[2] * 255.0) as u8;
            }
        }
    }
}

fn edge(a: [f32; 3], b: [f32; 3], c: [f32; 3]) -> f32 {
    (c[0] - a[0]) * (b[1] - a[1]) - (c[1] - a[1]) * (b[0] - a[0])
}

fn draw_line(rgb: &mut [u8], w: usize, h: usize, a: [f32; 2], b: [f32; 2], col: [f32; 3], width: i32) {
    let steps = ((b[0] - a[0]).abs().max((b[1] - a[1]).abs())).ceil().max(1.0) as i32;
    for i in 0..=steps {
        let t = i as f32 / steps as f32;
        let x = (a[0] + (b[0] - a[0]) * t).round() as i32;
        let y = (a[1] + (b[1] - a[1]) * t).round() as i32;
        for dy in -(width / 2)..=(width / 2) {
            for dx in -(width / 2)..=(width / 2) {
                let px = x + dx;
                let py = y + dy;
                if px < 0 || py < 0 || px as usize >= w || py as usize >= h {
                    continue;
                }
                let idx = py as usize * w + px as usize;
                rgb[idx * 3] = (col[0] * 255.0) as u8;
                rgb[idx * 3 + 1] = (col[1] * 255.0) as u8;
                rgb[idx * 3 + 2] = (col[2] * 255.0) as u8;
            }
        }
    }
}

fn encode_png(rgb: &[u8], width: u32, height: u32) -> Vec<u8> {
    let mut out = Vec::new();
    {
        let mut encoder = png::Encoder::new(&mut out, width, height);
        encoder.set_color(png::ColorType::Rgb);
        encoder.set_depth(png::BitDepth::Eight);
        let mut writer = encoder.write_header().expect("png header");
        writer.write_image_data(rgb).expect("png data");
    }
    out
}

/// A simple ortho view_proj (world→clip) looking from `eye` at `target`, +Z up
/// (or +Y if near-vertical), fitting `~span` world units across the viewport.
/// Used by lib tests and the demo binary — NOT a production camera: the +Z up
/// is this helper's own, for the Z-up scenes those demos build, and not the
/// application's world up (+Y — see the crate header).
pub fn test_view_proj(eye: [f32; 3], target: [f32; 3], width: f32, height: f32) -> [[f32; 4]; 4] {
    let eye = Vec3::from(eye);
    let target = Vec3::from(target);
    let fwd = target.sub(eye).normalized();
    let up = if fwd.z.abs() > 0.9 { Vec3::Y } else { Vec3::Z };
    crate::math::ortho_view_proj(eye, fwd, up, 6.0, width, height)
}
