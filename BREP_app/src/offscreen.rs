//! Render one egui pass into an offscreen texture and read it back.
//!
//! The desktop loop's on-demand pass (`desktop_loop`) never presents to the
//! window: a hidden Wayland surface gets no frame callbacks, and presenting to
//! it can block. A capture asked for during such a pass is drawn here instead,
//! with the window's own `egui_wgpu::RenderState`: the same device, the same
//! renderer (so its textures and the 3D viewport's paint-callback resources
//! are the ones the window draws with) and the same target format.
use eframe::egui_wgpu::{self, wgpu};

/// Upload the pass's new and changed textures to the shared renderer. The
/// window's next real frame never sees this pass's texture delta, so the
/// pass must apply it itself (a font atlas page added here is drawn there).
pub fn apply_texture_sets(render: &egui_wgpu::RenderState, delta: &egui::TexturesDelta) {
    let mut renderer = render.renderer.write();
    for (id, image) in &delta.set {
        renderer.update_texture(&render.device, &render.queue, *id, image);
    }
}

/// Free the textures the pass released, after anything that drew with them.
pub fn apply_texture_frees(render: &egui_wgpu::RenderState, delta: &egui::TexturesDelta) {
    let mut renderer = render.renderer.write();
    for id in &delta.free {
        renderer.free_texture(id);
    }
}

/// Draw `shapes` at `size_px` (`pixels_per_point`) over `clear` and return the
/// picture. Its texture delta must already be applied ([`apply_texture_sets`]).
pub fn capture(
    render: &egui_wgpu::RenderState,
    ctx: &egui::Context,
    shapes: Vec<egui::epaint::ClippedShape>,
    pixels_per_point: f32,
    size_px: [u32; 2],
    clear: [f32; 4],
) -> Result<egui::ColorImage, String> {
    let [width, height] = [size_px[0].max(1), size_px[1].max(1)];
    let screen = egui_wgpu::ScreenDescriptor { size_in_pixels: [width, height], pixels_per_point };
    let primitives = ctx.tessellate(shapes, pixels_per_point);
    let device = &render.device;
    let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor { label: Some("offscreen pass") });
    let texture = device.create_texture(&wgpu::TextureDescriptor {
        label: Some("offscreen pass"),
        size: wgpu::Extent3d { width, height, depth_or_array_layers: 1 },
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: render.target_format,
        usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::COPY_SRC,
        view_formats: &[],
    });
    let view = texture.create_view(&wgpu::TextureViewDescriptor::default());
    let user_buffers = {
        let mut renderer = render.renderer.write();
        let user_buffers = renderer.update_buffers(device, &render.queue, &mut encoder, &primitives, &screen);
        let [r, g, b, a] = clear;
        let mut pass = encoder
            .begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("offscreen pass"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &view,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color { r: r.into(), g: g.into(), b: b.into(), a: a.into() }),
                        store: wgpu::StoreOp::Store,
                    },
                    depth_slice: None,
                })],
                ..Default::default()
            })
            .forget_lifetime();
        renderer.render(&mut pass, &primitives, &screen);
        drop(pass);
        user_buffers
    };
    render.queue.submit(user_buffers.into_iter().chain(std::iter::once(encoder.finish())));
    read_back(render, &texture)
}

/// Copy `texture` to the CPU as straight RGBA, whatever channel order the
/// surface format uses.
fn read_back(render: &egui_wgpu::RenderState, texture: &wgpu::Texture) -> Result<egui::ColorImage, String> {
    let (width, height) = (texture.width(), texture.height());
    let unpadded = width as usize * 4;
    let align = wgpu::COPY_BYTES_PER_ROW_ALIGNMENT as usize;
    let padded = unpadded.div_ceil(align) * align;
    let device = &render.device;
    let buffer = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("offscreen read-back"),
        size: (padded * height as usize) as u64,
        usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });
    let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor { label: Some("offscreen read-back") });
    encoder.copy_texture_to_buffer(
        texture.as_image_copy(),
        wgpu::TexelCopyBufferInfo {
            buffer: &buffer,
            layout: wgpu::TexelCopyBufferLayout { offset: 0, bytes_per_row: Some(padded as u32), rows_per_image: None },
        },
        wgpu::Extent3d { width, height, depth_or_array_layers: 1 },
    );
    let submission = render.queue.submit(std::iter::once(encoder.finish()));
    let slice = buffer.slice(..);
    let (tx, rx) = std::sync::mpsc::channel();
    slice.map_async(wgpu::MapMode::Read, move |result| drop(tx.send(result)));
    device
        .poll(wgpu::PollType::Wait { submission_index: Some(submission), timeout: Some(std::time::Duration::from_secs(10)) })
        .map_err(|e| format!("offscreen capture: device poll: {e}"))?;
    rx.recv()
        .map_err(|_| "offscreen capture: the read-back never mapped".to_string())?
        .map_err(|e| format!("offscreen capture: map: {e}"))?;
    let bgra = matches!(texture.format(), wgpu::TextureFormat::Bgra8Unorm | wgpu::TextureFormat::Bgra8UnormSrgb);
    let data = slice.get_mapped_range();
    let mut pixels = Vec::with_capacity(width as usize * height as usize);
    for row in data.chunks_exact(padded) {
        for px in row[..unpadded].chunks_exact(4) {
            let (r, b) = if bgra { (px[2], px[0]) } else { (px[0], px[2]) };
            pixels.push(egui::Color32::from_rgba_premultiplied(r, px[1], b, px[3]));
        }
    }
    drop(data);
    buffer.unmap();
    Ok(egui::ColorImage {
        size: [width as usize, height as usize],
        source_size: egui::vec2(width as f32, height as f32),
        pixels,
    })
}
