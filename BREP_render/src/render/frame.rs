use super::*;

impl RenderCore {
    /// Render one frame into `resolve_view` (a single-sample view of the
    /// core's format). This is the whole engine core; every presentation shell
    /// funnels through it.
    pub fn render_to_view(
        &mut self,
        gpu_scene: &mut GpuScene,
        scene: &RenderScene,
        params: &FrameParams,
        resolve_view: &wgpu::TextureView,
    ) {
        let width = params.width.max(1);
        let height = params.height.max(1);
        self.ensure_targets(width, height);
        self.write_global_styles(params.settings);

        let globals = Globals {
            view_proj: params.camera.view_proj,
            viewport: [width as f32, height as f32, params.dpr.max(1e-3), 0.0],
            forward: [
                params.camera.forward[0],
                params.camera.forward[1],
                params.camera.forward[2],
                0.0,
            ],
        };
        self.queue
            .write_buffer(&self.globals_buf, 0, bytemuck::bytes_of(&globals));

        // World-axis helper (R20): three world-axis segments sized in CSS px.
        let axis_len = params.settings.axis_length_px as f64 * params.world_per_pixel;
        let draw_axes = params.settings.axis_length_px > 0.0 && axis_len.is_finite() && axis_len > 0.0;
        if draw_axes {
            let l = axis_len as f32;
            let segments = [
                EdgeInstance { p0: [0.0; 3], p1: [l, 0.0, 0.0] },
                EdgeInstance { p0: [0.0; 3], p1: [0.0, l, 0.0] },
                EdgeInstance { p0: [0.0; 3], p1: [0.0, 0.0, l] },
            ];
            match &gpu_scene.axis_buf {
                Some(buf) => self.queue.write_buffer(buf, 0, bytemuck::cast_slice(&segments)),
                None => {
                    gpu_scene.axis_buf = Some(self.device.create_buffer_init(
                        &wgpu::util::BufferInitDescriptor {
                            label: Some("world axes"),
                            contents: bytemuck::cast_slice(&segments),
                            usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
                        },
                    ));
                }
            }
        }

        // Refresh emphasis boundary buffers.
        let has_emphasis = !params.emphasis.is_empty();
        for solid in scene.solids() {
            if let Some(gpu_solid) = gpu_scene.solids.get_mut(&solid.name) {
                self.sync_boundary(gpu_solid, solid, params.emphasis);
            }
        }

        let bg = params.settings.background;
        let targets = self.targets.as_ref().expect("targets ensured");
        let mut encoder = self
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("brep-render frame"),
            });
        {
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("scene"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &targets.msaa_view,
                    depth_slice: None,
                    resolve_target: Some(resolve_view),
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color {
                            r: bg[0] as f64,
                            g: bg[1] as f64,
                            b: bg[2] as f64,
                            a: 1.0,
                        }),
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                    view: &targets.depth_view,
                    depth_ops: Some(wgpu::Operations {
                        load: wgpu::LoadOp::Clear(1.0),
                        store: wgpu::StoreOp::Discard,
                    }),
                    stencil_ops: None,
                }),
                timestamp_writes: None,
                occlusion_query_set: None,
                multiview_mask: None,
            });
            pass.set_bind_group(0, &self.globals_bind, &[]);

            let visible_solids: Vec<(&SolidDisplay, &GpuSolid)> = gpu_scene
                .order
                .iter()
                .filter_map(|name| {
                    let solid = scene.solid(name)?;
                    if !solid.visible {
                        return None;
                    }
                    Some((solid, gpu_scene.solids.get(name)?))
                })
                .collect();

            // 1a. Wireframe view: draw each solid's tessellated triangle mesh as
            //     a line list (base color, no fill), so the face triangles read
            //     as a wireframe. Picking is CPU ray-based (pick.rs), unaffected.
            //     A HIDDEN face skips its triangles' wire segments too, exactly as
            //     the shaded pass masks its triangle ranges — otherwise hiding a
            //     face (or the whole Faces group) would be a no-op in wireframe view.
            if params.settings.wireframe && params.settings.show_faces {
                pass.set_pipeline(&self.wire_pipeline);
                for (solid, gpu_solid) in &visible_solids {
                    if gpu_solid.wire_index_count == 0 {
                        continue;
                    }
                    pass.set_vertex_buffer(0, gpu_solid.vertex_buf.slice(..));
                    pass.set_index_buffer(
                        gpu_solid.wire_index_buf.slice(..),
                        wgpu::IndexFormat::Uint32,
                    );
                    pass.set_bind_group(1, &gpu_solid.base_style.bind, &[]);
                    // Fast path: no hidden face → one whole-buffer wire draw.
                    if !solid.visibility.any_face_hidden() {
                        pass.draw_indexed(0..gpu_solid.wire_index_count, 0, 0..1);
                        continue;
                    }
                    // Otherwise coalesce contiguous VISIBLE faces' wire ranges and
                    // skip the hidden ones. The wire buffer is a LINE LIST whose
                    // segments are grouped by face and de-duplicated inside each
                    // face (see `scene_sync::upload_solid`), so a face's slice is
                    // whatever that grouping produced — read it off the face's
                    // `wire_first`/`wire_count` rather than deriving it from the
                    // triangle range, which no longer predicts it.
                    let mut run: Option<(u32, u32)> = None; // first, count
                    let flush = |pass: &mut wgpu::RenderPass, run: &mut Option<(u32, u32)>| {
                        if let Some((first, count)) = run.take() {
                            if count > 0 {
                                pass.draw_indexed(first..first + count, 0, 0..1);
                            }
                        }
                    };
                    for (index, _face) in solid.faces.iter().enumerate() {
                        let range = &gpu_solid.faces[index];
                        if range.wire_count == 0 {
                            continue;
                        }
                        let first = range.wire_first;
                        let count = range.wire_count;
                        if !solid.visibility.is_face_visible(index) {
                            flush(&mut pass, &mut run);
                            continue;
                        }
                        match &mut run {
                            Some((run_first, run_count)) if *run_first + *run_count == first => {
                                *run_count += count;
                            }
                            _ => {
                                flush(&mut pass, &mut run);
                                run = Some((first, count));
                            }
                        }
                    }
                    flush(&mut pass, &mut run);
                }
            }

            // 1. Shaded faces, coalesced into index-range runs per emphasis
            //    state (base runs merge back into whole-solid draws). Skipped in
            //    wireframe mode (1a draws the triangle wireframe instead). Picking
            //    is CPU ray-based (pick.rs) and the overlay pass is separate.
            if !params.settings.wireframe && params.settings.show_faces {
                pass.set_pipeline(&self.mesh_pipeline);
                for (solid, gpu_solid) in &visible_solids {
                    if solid.mesh.indices.is_empty() {
                        continue;
                    }
                    pass.set_vertex_buffer(0, gpu_solid.vertex_buf.slice(..));
                    pass.set_index_buffer(gpu_solid.index_buf.slice(..), wgpu::IndexFormat::Uint32);
                    // Fast path: no emphasis AND no hidden faces → one whole-mesh
                    // draw. When any face is hidden we fall to the per-face loop
                    // below, which coalesces contiguous VISIBLE faces and skips
                    // the hidden ones' triangle ranges entirely.
                    // The whole-mesh fast path also needs every face to share
                    // the solid's colour — a solid with a per-face palette has
                    // to walk its faces to bind the right style per run.
                    let any_face_hidden = solid.visibility.any_face_hidden();
                    let any_face_colored = !gpu_solid.face_styles.is_empty();
                    if !has_emphasis && !any_face_hidden && !any_face_colored {
                        pass.set_bind_group(1, &gpu_solid.base_style.bind, &[]);
                        pass.draw_indexed(0..solid.mesh.indices.len() as u32, 0, 0..1);
                        continue;
                    }
                    // The run key is (emphasis state, face-colour slot). A
                    // selected or hovered face draws in the emphasis colour
                    // whatever colour the model gave it, so its slot is
                    // normalized away — two adjacent selected faces of
                    // different colours still coalesce into one draw.
                    let style_for = |state: EmphasisState, slot: Option<u32>| match state {
                        EmphasisState::Base => match slot.and_then(|s| gpu_solid.face_styles.get(s as usize)) {
                            Some(style) => &style.bind,
                            None => &gpu_solid.base_style.bind,
                        },
                        EmphasisState::Selected => &self.styles.face_selected.bind,
                        EmphasisState::Hovered => &self.styles.face_hovered.bind,
                    };
                    type FaceRun = (EmphasisState, Option<u32>, u32, u32); // state, slot, first, count
                    let mut run: Option<FaceRun> = None;
                    let flush = |pass: &mut wgpu::RenderPass, run: &mut Option<FaceRun>| {
                        if let Some((state, slot, first, count)) = run.take() {
                            if count > 0 {
                                pass.set_bind_group(1, style_for(state, slot), &[]);
                                pass.draw_indexed(first..first + count, 0, 0..1);
                            }
                        }
                    };
                    for (index, face) in solid.faces.iter().enumerate() {
                        let range = &gpu_solid.faces[index];
                        if range.index_count == 0 {
                            continue;
                        }
                        // Hidden face: skip its triangles and break the run so
                        // the surviving neighbours don't coalesce across the gap.
                        if !solid.visibility.is_face_visible(index) {
                            flush(&mut pass, &mut run);
                            continue;
                        }
                        let state = if has_emphasis {
                            params.emphasis.face_state(&solid.name, &face.name)
                        } else {
                            EmphasisState::Base
                        };
                        let slot = if state == EmphasisState::Base {
                            range.style
                        } else {
                            None
                        };
                        match &mut run {
                            Some((run_state, run_slot, first, count))
                                if *run_state == state
                                    && *run_slot == slot
                                    && *first + *count == range.first_index =>
                            {
                                *count += range.index_count;
                            }
                            _ => {
                                flush(&mut pass, &mut run);
                                run = Some((state, slot, range.first_index, range.index_count));
                            }
                        }
                    }
                    flush(&mut pass, &mut run);
                }
            }

            // 2. Occluded edge portions, dimmed (depth test inverted). Hidden
            //    edges skip their segments (their occluded portion too).
            if params.settings.show_edges
                && params.settings.show_occluded_edges
                && params.settings.hidden_edge_alpha > 0.0
            {
                pass.set_pipeline(&self.edge_hidden_pipeline);
                pass.set_bind_group(1, &self.styles.edge_hidden.bind, &[]);
                for (solid, gpu_solid) in &visible_solids {
                    let Some(edge_buf) = &gpu_solid.edge_buf else { continue };
                    if gpu_solid.occludable_edge_instances == 0 {
                        continue;
                    }
                    pass.set_vertex_buffer(0, edge_buf.slice(..));
                    if solid.visibility.any_edge_hidden() {
                        // Single style already bound: draw only visible edges.
                        draw_visible_edge_ranges(&mut pass, solid, gpu_solid, EdgeFilter::Real);
                    } else {
                        // Aux edges are not drawn occluded (`occludable_edge_instances`).
                        pass.draw(0..6, 0..gpu_solid.occludable_edge_instances);
                    }
                }
            }

            // 3. Visible edges, per-edge emphasis runs; hidden edges skipped.
            //    The whole pass is off when the Edges display toggle is.
            if params.settings.show_edges {
                let aux_shown = params.world_per_pixel <= AUX_EDGE_MAX_WORLD_PER_PIXEL;
                pass.set_pipeline(&self.edge_visible_pipeline);
                for (solid, gpu_solid) in &visible_solids {
                    let Some(edge_buf) = &gpu_solid.edge_buf else { continue };
                    if gpu_solid.edge_instances == 0 {
                        continue;
                    }
                    pass.set_vertex_buffer(0, edge_buf.slice(..));
                    let any_edge_hidden = solid.visibility.any_edge_hidden();
                    // The real edges are a prefix of the buffer and the AUX
                    // ones the rest; at rest the two draw in their own styles,
                    // and the aux ones only while the view is close enough
                    // for them to be told apart (`AUX_EDGE_MAX_WORLD_PER_PIXEL`).
                    let split = gpu_solid.occludable_edge_instances;
                    if !has_emphasis {
                        pass.set_bind_group(1, &self.styles.edge_base.bind, &[]);
                        if any_edge_hidden {
                            draw_visible_edge_ranges(&mut pass, solid, gpu_solid, EdgeFilter::Real);
                            if aux_shown {
                                pass.set_bind_group(1, &self.styles.edge_aux.bind, &[]);
                                draw_visible_edge_ranges(&mut pass, solid, gpu_solid, EdgeFilter::Aux);
                            }
                        } else {
                            if split > 0 {
                                pass.draw(0..6, 0..split);
                            }
                            if aux_shown && split < gpu_solid.edge_instances {
                                pass.set_bind_group(1, &self.styles.edge_aux.bind, &[]);
                                pass.draw(0..6, split..gpu_solid.edge_instances);
                            }
                        }
                        continue;
                    }
                    // An aux edge at rest keeps its own style; hovered or
                    // selected (its SOLID is), it emphasises like any edge.
                    let style_for = |(state, aux): (EmphasisState, bool)| match state {
                        EmphasisState::Base if aux => &self.styles.edge_aux.bind,
                        EmphasisState::Base => &self.styles.edge_base.bind,
                        EmphasisState::Selected => &self.styles.edge_selected.bind,
                        EmphasisState::Hovered => &self.styles.edge_hovered.bind,
                    };
                    let mut run: Option<((EmphasisState, bool), u32, u32)> = None;
                    let flush = |pass: &mut wgpu::RenderPass, run: &mut Option<((EmphasisState, bool), u32, u32)>| {
                        if let Some((state, first, count)) = run.take() {
                            if count > 0 {
                                pass.set_bind_group(1, style_for(state), &[]);
                                pass.draw(0..6, first..first + count);
                            }
                        }
                    };
                    for (index, edge) in solid.edges.iter().enumerate() {
                        let range = &gpu_solid.edges[index];
                        if range.instance_count == 0 {
                            continue;
                        }
                        // Hidden edge: skip its segments and break the run.
                        if !solid.visibility.is_edge_visible(index) || (edge.aux && !aux_shown) {
                            flush(&mut pass, &mut run);
                            continue;
                        }
                        let state = (params.emphasis.edge_state(&solid.name, &edge.name), edge.aux);
                        match &mut run {
                            Some((run_state, first, count))
                                if *run_state == state
                                    && *first + *count == range.first_instance =>
                            {
                                *count += range.instance_count;
                            }
                            _ => {
                                flush(&mut pass, &mut run);
                                run = Some((state, range.first_instance, range.instance_count));
                            }
                        }
                    }
                    flush(&mut pass, &mut run);
                }
            }

            // 4. Selected/hovered face boundary outlines.
            if has_emphasis {
                // Bind the pipeline explicitly rather than inheriting whatever
                // the passes above left set: pass 3 binds this same pipeline,
                // but only when `show_edges` is on. With edges hidden the
                // inherited pipeline was the mesh/wireframe one, whose slot 0
                // is a Vertex-step `MeshVertex` — so the `draw(0..6, ..)` below
                // read this Instance-step `EdgeInstance` buffer as vertices and
                // overran it (a 4-segment boundary is 4 "vertices" at the same
                // 24-byte stride), which wgpu reports as a fatal validation error.
                pass.set_pipeline(&self.edge_visible_pipeline);
                pass.set_bind_group(1, &self.styles.boundary.bind, &[]);
                for (_, gpu_solid) in &visible_solids {
                    let Some(boundary) = &gpu_solid.boundary else { continue };
                    let Some(buf) = &boundary.buf else { continue };
                    if boundary.count == 0 {
                        continue;
                    }
                    pass.set_vertex_buffer(0, buf.slice(..));
                    pass.draw(0..6, 0..boundary.count);
                }
            }

            // 5. Vertex points; hidden vertices skip their point sprite.
            if params.settings.show_vertices && params.settings.vertex_size_px > 0.0 {
                pass.set_pipeline(&self.point_pipeline);
                for (solid, gpu_solid) in &visible_solids {
                    let Some(point_buf) = &gpu_solid.point_buf else { continue };
                    if gpu_solid.point_count == 0 {
                        continue;
                    }
                    // A fully-hidden points group draws nothing — skip before touching
                    // the per-vertex loop below. When ALL vertices are hidden that loop
                    // would still iterate every one of them each frame; the O(N)/frame
                    // cost is invisible on native but throttles the WebGL backend on
                    // point-heavy models (the "can't spin after hiding points" case).
                    if solid.visibility.all_vertices_hidden(solid.vertices.len()) {
                        continue;
                    }
                    pass.set_vertex_buffer(0, point_buf.slice(..));
                    let any_vertex_hidden = solid.visibility.any_vertex_hidden();
                    if has_emphasis || any_vertex_hidden {
                        let tol = 1e-9_f64.max(params.world_per_pixel * 1e-3);
                        let mut base_run: Option<(u32, u32)> = None;
                        let mut emphasized: Vec<(EmphasisState, u32)> = Vec::new();
                        for (index, vertex) in solid.vertices.iter().enumerate() {
                            // Hidden vertex: skip its point and break the run.
                            if !solid.visibility.is_vertex_visible(index) {
                                if let Some((first, count)) = base_run.take() {
                                    pass.set_bind_group(1, &self.styles.point_base.bind, &[]);
                                    pass.draw(0..6, first..first + count);
                                }
                                continue;
                            }
                            let state = if has_emphasis {
                                params.emphasis.vertex_state(&solid.name, vertex.position, tol)
                            } else {
                                EmphasisState::Base
                            };
                            if state == EmphasisState::Base {
                                match &mut base_run {
                                    Some((first, count)) if *first + *count == index as u32 => {
                                        *count += 1
                                    }
                                    _ => {
                                        if let Some((first, count)) = base_run.take() {
                                            pass.set_bind_group(1, &self.styles.point_base.bind, &[]);
                                            pass.draw(0..6, first..first + count);
                                        }
                                        base_run = Some((index as u32, 1));
                                    }
                                }
                            } else {
                                emphasized.push((state, index as u32));
                            }
                        }
                        if let Some((first, count)) = base_run {
                            pass.set_bind_group(1, &self.styles.point_base.bind, &[]);
                            pass.draw(0..6, first..first + count);
                        }
                        for (state, index) in emphasized {
                            let style = match state {
                                EmphasisState::Selected => &self.styles.point_selected.bind,
                                _ => &self.styles.point_hovered.bind,
                            };
                            pass.set_bind_group(1, style, &[]);
                            pass.draw(0..6, index..index + 1);
                        }
                    } else {
                        pass.set_bind_group(1, &self.styles.point_base.bind, &[]);
                        pass.draw(0..6, 0..gpu_solid.point_count);
                    }
                }
            }

            // 6. World axes on top of nothing special (normal depth test).
            if draw_axes {
                if let Some(axis_buf) = &gpu_scene.axis_buf {
                    pass.set_pipeline(&self.edge_visible_pipeline);
                    pass.set_vertex_buffer(0, axis_buf.slice(..));
                    for (index, style) in [
                        &self.styles.axis_x,
                        &self.styles.axis_y,
                        &self.styles.axis_z,
                    ]
                    .iter()
                    .enumerate()
                    {
                        pass.set_bind_group(1, &style.bind, &[]);
                        let i = index as u32;
                        pass.draw(0..6, i..i + 1);
                    }
                }
            }
        }

        // --- Overlay-widget passes: the brep-gizmos overlay drawn
        //     over the solids in a depth-cleared pass so widgets read on top.
        //     The main overlay uses the scene camera + full viewport; the
        //     ViewCube uses its own mini-camera + corner viewport.
        if let Some(overlay) = params.overlay {
            let make_tris = |ov: &brep_gizmos::Overlay| -> Option<wgpu::Buffer> {
                let verts = overlay_tri_verts(ov);
                (!verts.is_empty()).then(|| {
                    self.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                        label: Some("overlay tris"),
                        contents: bytemuck::cast_slice(&verts),
                        usage: wgpu::BufferUsages::VERTEX,
                    })
                })
            };
            let make_lines = |ov: &brep_gizmos::Overlay| -> Option<wgpu::Buffer> {
                let insts = overlay_line_insts(ov);
                (!insts.is_empty()).then(|| {
                    self.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                        label: Some("overlay lines"),
                        contents: bytemuck::cast_slice(&insts),
                        usage: wgpu::BufferUsages::VERTEX,
                    })
                })
            };

            let main_tri_count = overlay.main.tris.len() as u32;
            let main_line_count = (overlay.main.lines.len() / 2) as u32;
            let main_tri_buf = make_tris(&overlay.main);
            let main_line_buf = make_lines(&overlay.main);
            if main_tri_buf.is_some() || main_line_buf.is_some() {
                let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                    label: Some("overlay-main"),
                    color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                        view: &targets.msaa_view,
                        depth_slice: None,
                        resolve_target: Some(resolve_view),
                        ops: wgpu::Operations {
                            load: wgpu::LoadOp::Load,
                            store: wgpu::StoreOp::Store,
                        },
                    })],
                    depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                        view: &targets.depth_view,
                        depth_ops: Some(wgpu::Operations {
                            load: wgpu::LoadOp::Clear(1.0),
                            store: wgpu::StoreOp::Discard,
                        }),
                        stencil_ops: None,
                    }),
                    timestamp_writes: None,
                    occlusion_query_set: None,
                    multiview_mask: None,
                });
                pass.set_bind_group(0, &self.globals_bind, &[]);
                pass.set_bind_group(1, &self.styles.overlay_line.bind, &[]);
                if let Some(buf) = &main_tri_buf {
                    pass.set_vertex_buffer(0, buf.slice(..));
                    // The datum/construction PLANE tris come FIRST; draw them with
                    // depth-write OFF so a translucent plane never occludes the
                    // gizmo/dimension tris that follow. The rest keep depth-write
                    // so gizmos self-occlude correctly.
                    let plane_verts = (overlay.plane_tri_verts as u32).min(main_tri_count);
                    if plane_verts > 0 {
                        pass.set_pipeline(&self.overlay_tri_nodepth_pipeline);
                        pass.draw(0..plane_verts, 0..1);
                    }
                    if plane_verts < main_tri_count {
                        pass.set_pipeline(&self.overlay_tri_pipeline);
                        pass.draw(plane_verts..main_tri_count, 0..1);
                    }
                }
                if let Some(buf) = &main_line_buf {
                    pass.set_pipeline(&self.overlay_line_pipeline);
                    pass.set_vertex_buffer(0, buf.slice(..));
                    pass.draw(0..6, 0..main_line_count);
                }
            }

            if let Some(vc) = &overlay.viewcube {
                let dpr = params.dpr.max(1e-3);
                let mut x = (vc.rect_css[0] * dpr).max(0.0);
                let mut y = (vc.rect_css[1] * dpr).max(0.0);
                let mut w = (vc.rect_css[2] * dpr).max(1.0);
                let mut h = (vc.rect_css[3] * dpr).max(1.0);
                // Clamp the corner viewport to the framebuffer.
                w = w.min(width as f32 - x).max(1.0);
                h = h.min(height as f32 - y).max(1.0);
                x = x.min(width as f32 - w).max(0.0);
                y = y.min(height as f32 - h).max(0.0);

                let vc_globals = Globals {
                    view_proj: vc.view_proj,
                    viewport: [w, h, dpr, 0.0],
                    forward: [vc.forward[0], vc.forward[1], vc.forward[2], 0.0],
                };
                self.queue
                    .write_buffer(&self.vc_globals_buf, 0, bytemuck::bytes_of(&vc_globals));

                let vc_tri_count = vc.overlay.tris.len() as u32;
                let vc_line_count = (vc.overlay.lines.len() / 2) as u32;
                let vc_tri_buf = make_tris(&vc.overlay);
                let vc_line_buf = make_lines(&vc.overlay);
                if vc_tri_buf.is_some() || vc_line_buf.is_some() {
                    let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                        label: Some("overlay-viewcube"),
                        color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                            view: &targets.msaa_view,
                            depth_slice: None,
                            resolve_target: Some(resolve_view),
                            ops: wgpu::Operations {
                                load: wgpu::LoadOp::Load,
                                store: wgpu::StoreOp::Store,
                            },
                        })],
                        depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                            view: &targets.depth_view,
                            depth_ops: Some(wgpu::Operations {
                                load: wgpu::LoadOp::Clear(1.0),
                                store: wgpu::StoreOp::Discard,
                            }),
                            stencil_ops: None,
                        }),
                        timestamp_writes: None,
                        occlusion_query_set: None,
                        multiview_mask: None,
                    });
                    pass.set_viewport(x, y, w, h, 0.0, 1.0);
                    pass.set_scissor_rect(x as u32, y as u32, w as u32, h as u32);
                    pass.set_bind_group(0, &self.vc_globals_bind, &[]);
                    pass.set_bind_group(1, &self.styles.overlay_line.bind, &[]);
                    if let Some(buf) = &vc_tri_buf {
                        pass.set_pipeline(&self.overlay_tri_pipeline);
                        pass.set_vertex_buffer(0, buf.slice(..));
                        pass.draw(0..vc_tri_count, 0..1);
                    }
                    if let Some(buf) = &vc_line_buf {
                        pass.set_pipeline(&self.overlay_line_pipeline);
                        pass.set_vertex_buffer(0, buf.slice(..));
                        pass.draw(0..6, 0..vc_line_count);
                    }
                }
            }
        }

        self.queue.submit([encoder.finish()]);
    }

    /// Headless capture (R32/R34): render the scene and return PNG bytes
    /// (8-bit RGB, no ancillary chunks — deterministic, R33).
    #[cfg(not(target_arch = "wasm32"))]
    pub fn render_to_png(
        &mut self,
        scene: &RenderScene,
        camera: &Camera,
        width: u32,
        height: u32,
    ) -> Result<Vec<u8>, String> {
        let settings = RenderSettings::artifact();
        let emphasis = Emphasis::default();
        let mut gpu_scene = self.upload_scene_with(scene, &settings);
        let params = FrameParams {
            camera,
            width,
            height,
            dpr: 1.0,
            settings: &settings,
            emphasis: &emphasis,
            world_per_pixel: 0.0,
            overlay: None,
        };
        let resolve = self.device.create_texture(&wgpu::TextureDescriptor {
            label: Some("resolve"),
            size: wgpu::Extent3d {
                width,
                height,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: self.format,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::COPY_SRC,
            view_formats: &[],
        });
        let resolve_view = resolve.create_view(&Default::default());
        self.render_to_view(&mut gpu_scene, scene, &params, &resolve_view);

        // Readback: rows padded to 256 bytes per wgpu's copy alignment.
        let bytes_per_row = (width * 4).div_ceil(256) * 256;
        let readback = self.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("readback"),
            size: bytes_per_row as u64 * height as u64,
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });
        let mut encoder = self
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("readback"),
            });
        encoder.copy_texture_to_buffer(
            wgpu::TexelCopyTextureInfo {
                texture: &resolve,
                mip_level: 0,
                origin: wgpu::Origin3d::ZERO,
                aspect: wgpu::TextureAspect::All,
            },
            wgpu::TexelCopyBufferInfo {
                buffer: &readback,
                layout: wgpu::TexelCopyBufferLayout {
                    offset: 0,
                    bytes_per_row: Some(bytes_per_row),
                    rows_per_image: None,
                },
            },
            wgpu::Extent3d {
                width,
                height,
                depth_or_array_layers: 1,
            },
        );
        self.queue.submit([encoder.finish()]);

        let slice = readback.slice(..);
        let (sender, receiver) = std::sync::mpsc::channel();
        slice.map_async(wgpu::MapMode::Read, move |result| {
            let _ = sender.send(result);
        });
        self.device
            .poll(wgpu::PollType::wait_indefinitely())
            .map_err(|error| format!("wgpu poll: {error:?}"))?;
        receiver
            .recv()
            .map_err(|_| "readback callback dropped".to_string())?
            .map_err(|error| format!("readback map failed: {error:?}"))?;

        let data = slice.get_mapped_range();
        let mut rgb = Vec::with_capacity((width * height * 3) as usize);
        for row in 0..height {
            let start = (row * bytes_per_row) as usize;
            for col in 0..width as usize {
                let px = start + col * 4;
                rgb.extend_from_slice(&data[px..px + 3]);
            }
        }
        drop(data);
        readback.unmap();

        encode_png(&rgb, width, height)
    }
}

/// Issue draw calls for a solid's VISIBLE edge instances only, coalescing
/// contiguous edge ranges into as few `draw`s as possible. Used by the
/// single-style edge passes (occluded/hidden, and the no-emphasis visible pass)
/// when the solid has any hidden edge — the caller has already bound the style.
/// The coarsest view, in world millimetres per CSS pixel, that still draws AUX
/// edges — a board's copper outline. Past it a 0.2 mm track is under two
/// pixels, so its outline would cover it instead of edging it and a dense track
/// field reads as one slab; and a board seen whole at that scale is exactly
/// when the most of its thousands of outline segments are on screen at once.
/// Zooming in brings them back.
const AUX_EDGE_MAX_WORLD_PER_PIXEL: f64 = 0.1;

/// Which of a solid's edges a [`draw_visible_edge_ranges`] call draws.
#[derive(Clone, Copy, PartialEq, Eq)]
enum EdgeFilter {
    /// The solid's own edges, not its display-only (aux) ones.
    Real,
    /// Only the display-only ones.
    Aux,
}

fn draw_visible_edge_ranges(
    pass: &mut wgpu::RenderPass<'_>,
    solid: &SolidDisplay,
    gpu_solid: &GpuSolid,
    filter: EdgeFilter,
) {
    let mut run: Option<(u32, u32)> = None; // first_instance, count
    for (index, edge) in solid.edges.iter().enumerate() {
        let range = &gpu_solid.edges[index];
        if range.instance_count == 0 {
            continue;
        }
        if !solid.visibility.is_edge_visible(index) || edge.aux != (filter == EdgeFilter::Aux) {
            if let Some((first, count)) = run.take() {
                pass.draw(0..6, first..first + count);
            }
            continue;
        }
        match &mut run {
            Some((first, count)) if *first + *count == range.first_instance => {
                *count += range.instance_count;
            }
            _ => {
                if let Some((first, count)) = run.take() {
                    pass.draw(0..6, first..first + count);
                }
                run = Some((range.first_instance, range.instance_count));
            }
        }
    }
    if let Some((first, count)) = run {
        pass.draw(0..6, first..first + count);
    }
}

/// Convert a gizmo `Overlay`'s triangles into GPU vertices (per-vertex color).
fn overlay_tri_verts(ov: &brep_gizmos::Overlay) -> Vec<OverlayTriVertex> {
    ov.tris
        .iter()
        .map(|v| OverlayTriVertex {
            position: v.pos,
            normal: v.normal,
            color: v.color,
        })
        .collect()
}

/// Convert a gizmo `Overlay`'s line segments (vertex pairs) into GPU instances
/// (per-instance color; both endpoints of a gizmo segment share a color).
fn overlay_line_insts(ov: &brep_gizmos::Overlay) -> Vec<OverlayLineInstance> {
    ov.lines
        .chunks_exact(2)
        .map(|pair| OverlayLineInstance {
            p0: pair[0].pos,
            p1: pair[1].pos,
            color: pair[0].color,
        })
        .collect()
}
