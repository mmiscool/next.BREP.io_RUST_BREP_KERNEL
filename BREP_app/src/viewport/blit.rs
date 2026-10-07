use super::*;

impl egui_wgpu::CallbackTrait for ViewportCallback {
    fn paint(
        &self,
        _info: egui::PaintCallbackInfo,
        render_pass: &mut wgpu::RenderPass<'static>,
        resources: &egui_wgpu::CallbackResources,
    ) {
        let Some(vp) = resources.get::<ViewportPaint>() else {
            return;
        };
        // egui has already set the render pass viewport to our panel rect, so a
        // clip-space fullscreen triangle fills exactly the viewport.
        render_pass.set_pipeline(&vp.pipeline);
        render_pass.set_bind_group(0, &vp.bind_group, &[]);
        render_pass.draw(0..3, 0..1);
    }
}

impl Viewport {
    /// Build the viewport's GPU resources from eframe's SHARED render state. We
    /// render in the engine's native `COLOR_FORMAT` (Rgba8Unorm) — exactly what
    /// the headless artifact path uses, so colors are known-good — and the blit
    /// converts into eframe's actual target format.
    pub fn new(render_state: &egui_wgpu::RenderState) -> Self {
        let device = render_state.device.clone();
        let queue = render_state.queue.clone();
        let core = RenderCore::new(device.clone(), queue, COLOR_FORMAT);
        let gpu_scene = GpuScene::default();

        // --- blit pipeline: offscreen 3D texture -> egui's frame target -------
        let target_format = render_state.target_format;
        let blit_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("brep-app viewport blit"),
            entries: &[
                wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Texture {
                        sample_type: wgpu::TextureSampleType::Float { filterable: true },
                        view_dimension: wgpu::TextureViewDimension::D2,
                        multisampled: false,
                    },
                    count: None,
                },
                wgpu::BindGroupLayoutEntry {
                    binding: 1,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::Filtering),
                    count: None,
                },
            ],
        });
        let blit_sampler = device.create_sampler(&wgpu::SamplerDescriptor {
            label: Some("brep-app viewport sampler"),
            mag_filter: wgpu::FilterMode::Nearest,
            min_filter: wgpu::FilterMode::Nearest,
            ..Default::default()
        });
        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("brep-app blit"),
            source: wgpu::ShaderSource::Wgsl(blit_wgsl(target_format.is_srgb()).into()),
        });
        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("brep-app blit layout"),
            bind_group_layouts: &[Some(&blit_layout)],
            immediate_size: 0,
        });
        let blit_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("brep-app blit pipeline"),
            layout: Some(&pipeline_layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: Some("vs"),
                compilation_options: Default::default(),
                buffers: &[],
            },
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: Some("fs"),
                compilation_options: Default::default(),
                targets: &[Some(wgpu::ColorTargetState {
                    format: target_format,
                    blend: None,
                    write_mask: wgpu::ColorWrites::ALL,
                })],
            }),
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::TriangleList,
                ..Default::default()
            },
            depth_stencil: None,
            // egui's frame render pass is single-sample (web painter + native
            // with multisampling=0), so the blit pipeline must match.
            multisample: wgpu::MultisampleState::default(),
            multiview_mask: None,
            cache: None,
        });

        Self {
            core,
            gpu_scene,
            egui_renderer: render_state.renderer.clone(),
            blit_layout,
            blit_sampler,
            blit_pipeline,
            offscreen: None,
            sheet: super::sheet::SheetViewport::default(),
            ecad: super::ecad::EcadViewport::default(),
            dragging: false,
            touch: Default::default(),
            touch_suppressed: false,
            gizmo_dragging: false,
            component_gizmo_dragging: false,
            dim_dragging: None,
            sketch_dragging: false,
            sketch_handdrawing: false,
            last_rect: None,
            candidate_popup: None,
            candidate_popup_fresh: false,
            hover_lit_since: None,
            hover_last_pos: None,
            candidate_popup_rect: None,
            candidate_hits: Vec::new(),
            editing_dim: None,
            dim_edit_fresh: false,
            editing_feature_dim: None,
            feature_dim_edit_fresh: false,
            constraint_dragging: false,
            constraint_label_hovered: None,
            pmi_label_hovered: None,
            pmi_label_dragging: None,
            pmi_label_hits: Vec::new(),
            pmi_label_menu: None,
            pmi_menu_hits: std::collections::HashMap::new(),
        }
    }

    /// (Re)create the offscreen texture at `w`x`h` physical px and refresh the
    /// callback's blit bind group in egui's resource map.
    pub(super) fn ensure_offscreen(&mut self, w: u32, h: u32) {
        if let Some(off) = &self.offscreen {
            if off.w == w && off.h == h {
                return;
            }
        }
        let texture = self.core.device.create_texture(&wgpu::TextureDescriptor {
            label: Some("brep-app viewport offscreen"),
            size: wgpu::Extent3d {
                width: w,
                height: h,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: COLOR_FORMAT,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::TEXTURE_BINDING,
            view_formats: &[],
        });
        let view = texture.create_view(&Default::default());
        let bind_group = self.core.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("brep-app viewport blit bind group"),
            layout: &self.blit_layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: wgpu::BindingResource::TextureView(&view),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: wgpu::BindingResource::Sampler(&self.blit_sampler),
                },
            ],
        });
        // Publish the fresh pipeline+bind-group for the paint callback to read.
        self.egui_renderer
            .write()
            .callback_resources
            .insert(ViewportPaint {
                pipeline: self.blit_pipeline.clone(),
                bind_group,
            });
        self.offscreen = Some(Offscreen { view, w, h });
    }

    /// Render the 3D scene into the offscreen texture via the engine's render
    /// core (its own MSAA + submit, on the shared queue). Runs only when the
    /// engine is dirty (R22 on-demand).
    pub(super) fn render_viewport(&mut self, phys_w: u32, phys_h: u32, ppp: f32, state: &mut EngineState) {
        let Some(off) = &self.offscreen else { return };
        {
            let _span = crate::perf::span(crate::perf::Phase::Sync);
            self.core.sync_scene(
                &mut self.gpu_scene,
                &state.scene,
                &state.settings,
                state.settings_generation,
            );
        }
        // Fit near/far to everything drawn (solids + the pushed overlay + the FULL
        // widget overlay's world bounds + the origin) and resolve the camera in one
        // shared step — see `EngineState::fit_camera_and_overlay`. Folding the full
        // overlay in stops construction geometry (datum planes, world axes, gizmos)
        // and an editing sketch from clipping against the solids-only bounds.
        let (camera, overlay) = {
            let _span = crate::perf::span(crate::perf::Phase::Fit);
            state.fit_camera_and_overlay()
        };
        let params = FrameParams {
            camera: &camera,
            width: phys_w,
            height: phys_h,
            dpr: ppp,
            settings: &state.settings,
            emphasis: &state.emphasis,
            world_per_pixel: state.camera.world_per_pixel(),
            overlay: overlay.as_ref(),
        };
        {
            let _span = crate::perf::span(crate::perf::Phase::Draw);
            self.core
                .render_to_view(&mut self.gpu_scene, &state.scene, &params, &off.view);
        }
        state.dirty = false;
    }
}

/// The blit shader. Fullscreen triangle; the fragment linearizes the (already
/// display-encoded) engine output when egui's target is sRGB, so the hardware
/// re-encode reproduces the engine's exact colors; otherwise it passes through.
fn blit_wgsl(target_is_srgb: bool) -> String {
    let frag_body = if target_is_srgb {
        // sampled texels are sRGB-encoded display values -> linearize so the
        // sRGB target's write-encode round-trips them back.
        r#"
    let c = textureSample(tex, samp, in.uv);
    let rgb = srgb_to_linear(c.rgb);
    return vec4<f32>(rgb, c.a);
"#
    } else {
        r#"
    return textureSample(tex, samp, in.uv);
"#
    };
    format!(
        r#"
@group(0) @binding(0) var tex: texture_2d<f32>;
@group(0) @binding(1) var samp: sampler;

struct VsOut {{
    @builtin(position) pos: vec4<f32>,
    @location(0) uv: vec2<f32>,
}};

@vertex
fn vs(@builtin(vertex_index) vi: u32) -> VsOut {{
    var p = array<vec2<f32>, 3>(
        vec2<f32>(-1.0, -1.0),
        vec2<f32>( 3.0, -1.0),
        vec2<f32>(-1.0,  3.0),
    );
    var out: VsOut;
    let xy = p[vi];
    out.pos = vec4<f32>(xy, 0.0, 1.0);
    // Framebuffer origin is top-left: map clip +y (top) -> uv.y 0.
    out.uv = vec2<f32>((xy.x + 1.0) * 0.5, (1.0 - xy.y) * 0.5);
    return out;
}}

fn srgb_to_linear(c: vec3<f32>) -> vec3<f32> {{
    let lo = c / 12.92;
    let hi = pow((c + 0.055) / 1.055, vec3<f32>(2.4));
    return select(hi, lo, c <= vec3<f32>(0.04045));
}}

@fragment
fn fs(in: VsOut) -> @location(0) vec4<f32> {{{frag_body}}}
"#
    )
}
