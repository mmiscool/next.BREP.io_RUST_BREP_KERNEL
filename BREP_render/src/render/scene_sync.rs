use super::*;

/// An AUX edge's width at rest, in CSS px ([`GlobalStyles::edge_aux`]).
const AUX_EDGE_WIDTH_PX: f32 = 1.0;

impl RenderCore {
    /// The base face color of one solid under the active settings, in LINEAR
    /// space (converted with f64 precision so artifact bytes stay stable); a
    /// display with an `opacity` carries it as the alpha (the blended pass
    /// reads it, the opaque pass skips such a solid).
    fn face_base_color(solid: &SolidDisplay, settings: &RenderSettings) -> Rgba {
        let mut color = Self::face_base_color_opaque(solid, settings);
        if let Some(opacity) = solid.opacity {
            color[3] = opacity.clamp(0.0, 1.0);
        }
        color
    }

    fn face_base_color_opaque(solid: &SolidDisplay, settings: &RenderSettings) -> Rgba {
        if let Some(rgb) = solid.color_override {
            return Self::opaque_linear(rgb);
        }
        match settings.face_color_mode {
            FaceColorMode::HashedBySolid => {
                let srgb = solid_color_srgb(&solid.name);
                [
                    srgb_to_linear(srgb[0]) as f32,
                    srgb_to_linear(srgb[1]) as f32,
                    srgb_to_linear(srgb[2]) as f32,
                    1.0,
                ]
            }
            FaceColorMode::Uniform => {
                let c = settings.face_color;
                [
                    srgb_to_linear(c[0] as f64) as f32,
                    srgb_to_linear(c[1] as f64) as f32,
                    srgb_to_linear(c[2] as f64) as f32,
                    c[3],
                ]
            }
        }
    }

    /// One sRGB model colour as an opaque LINEAR `Rgba` — the conversion the
    /// solid's base style and the per-face palette must share.
    fn opaque_linear(rgb: [f32; 3]) -> Rgba {
        [
            srgb_to_linear(rgb[0] as f64) as f32,
            srgb_to_linear(rgb[1] as f64) as f32,
            srgb_to_linear(rgb[2] as f64) as f32,
            1.0,
        ]
    }

    /// The shading params every face style carries (flat-vs-smooth in `.x`).
    fn style_params(settings: &RenderSettings) -> [f32; 4] {
        [if settings.flat_shading { 1.0 } else { 0.0 }, 0.0, 0.0, 0.0]
    }

    /// Build this solid's PER-FACE style palette, de-duplicated by colour, and
    /// the per-face index into it.
    ///
    /// Faces sharing a colour share one uniform buffer, so an imported body
    /// painted in three colours costs three buffers however many faces it has —
    /// and the draw loop can still coalesce a contiguous run of same-coloured
    /// faces into one call. A solid with no per-face colours returns an EMPTY
    /// palette, which is what re-arms the whole-mesh fast path.
    fn face_style_palette(
        &self,
        solid: &SolidDisplay,
        settings: &RenderSettings,
    ) -> (Vec<StyleBuf>, Vec<Option<u32>>) {
        let mut styles: Vec<StyleBuf> = Vec::new();
        let mut colors: Vec<[f32; 3]> = Vec::new();
        let mut per_face: Vec<Option<u32>> = Vec::with_capacity(solid.faces.len());
        let params = Self::style_params(settings);
        for face in &solid.faces {
            let Some(rgb) = face.color_override else {
                per_face.push(None);
                continue;
            };
            let slot = match colors.iter().position(|c| *c == rgb) {
                Some(slot) => slot,
                None => {
                    let style = StyleBuf::new(
                        &self.device,
                        &self.style_layout,
                        solid.name.as_str(),
                    );
                    style.write(&self.queue, Self::opaque_linear(rgb), params);
                    styles.push(style);
                    colors.push(rgb);
                    colors.len() - 1
                }
            };
            per_face.push(Some(slot as u32));
        }
        (styles, per_face)
    }

    /// The vertex buffer's contents: positions and normals.
    fn mesh_vertices(solid: &SolidDisplay) -> Vec<MeshVertex> {
        solid
            .mesh
            .positions
            .iter()
            .zip(&solid.mesh.normals)
            .map(|(&position, &normal)| MeshVertex { position, normal })
            .collect()
    }

    /// The edge buffer's contents (one instance per polyline segment) and each
    /// edge's instance range.
    fn edge_segments(solid: &SolidDisplay) -> (Vec<EdgeInstance>, Vec<EdgeRange>) {
        let mut segments: Vec<EdgeInstance> = Vec::new();
        let mut edges = Vec::with_capacity(solid.edges.len());
        for edge in &solid.edges {
            let first_instance = segments.len() as u32;
            for pair in edge.polyline.windows(2) {
                segments.push(EdgeInstance {
                    p0: pair[0],
                    p1: pair[1],
                });
            }
            edges.push(EdgeRange {
                first_instance,
                instance_count: segments.len() as u32 - first_instance,
            });
        }
        (segments, edges)
    }

    /// The point buffer's contents: one instance per vertex.
    fn point_instances(solid: &SolidDisplay) -> Vec<PointInstance> {
        solid
            .vertices
            .iter()
            .map(|v| PointInstance {
                center: [
                    v.position[0] as f32,
                    v.position[1] as f32,
                    v.position[2] as f32,
                ],
            })
            .collect()
    }

    /// Re-pose a solid whose buffers hold the same SHAPE (a rigid move): rewrite
    /// its vertex, edge and point buffers in place. `false` (nothing written)
    /// when any count differs, so the caller rebuilds instead.
    fn repose_solid(&self, existing: &GpuSolid, solid: &SolidDisplay) -> bool {
        let vertices = Self::mesh_vertices(solid);
        let (segments, edges) = Self::edge_segments(solid);
        let points = Self::point_instances(solid);
        let same = vertices.len() as u32 == existing.vertex_count
            && segments.len() as u32 == existing.edge_instances
            && edges.len() == existing.edges.len()
            && points.len() as u32 == existing.point_count
            && solid.faces.len() == existing.faces.len()
            && existing.edge_buf.is_some() == !segments.is_empty()
            && existing.point_buf.is_some() == !points.is_empty();
        if !same {
            return false;
        }
        self.queue.write_buffer(&existing.vertex_buf, 0, bytemuck::cast_slice(&vertices));
        if let Some(buf) = &existing.edge_buf {
            self.queue.write_buffer(buf, 0, bytemuck::cast_slice(&segments));
        }
        if let Some(buf) = &existing.point_buf {
            self.queue.write_buffer(buf, 0, bytemuck::cast_slice(&points));
        }
        true
    }

    fn upload_solid(&self, solid: &SolidDisplay, settings: &RenderSettings) -> GpuSolid {
        let vertices = Self::mesh_vertices(solid);
        let vertex_buf = self
            .device
            .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some(solid.name.as_str()),
                contents: bytemuck::cast_slice(&vertices),
                // COPY_DST: a rigid re-pose rewrites it in place.
                usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
            });
        let index_buf = self
            .device
            .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some(solid.name.as_str()),
                contents: bytemuck::cast_slice(&solid.mesh.indices),
                usage: wgpu::BufferUsages::INDEX,
            });
        // Wireframe line-list, built PER FACE and de-duplicated inside each face.
        //
        // The obvious expansion — every triangle (a,b,c) → (a,b),(b,c),(c,a) —
        // emits each of a face's INTERIOR edges twice, once from each of the two
        // triangles that share it. That is two-thirds of the segments on a typical
        // tessellation: twice the index data to upload and hold, and twice the
        // vertex work per frame to draw two collinear lines on top of each other.
        // The wire pipeline does not blend (`color_target(None)` in `pipelines.rs`),
        // so the second draw of a segment lands on the same pixels in the same
        // colour at the same depth — it was never visible.
        //
        // De-duplicating WITHIN a face, rather than across the whole mesh, is what
        // keeps each face's segments CONTIGUOUS, which is what lets the hidden-face
        // path below address them by range the way the shaded path addresses
        // triangle ranges. The seam between two faces stays doubled (once per
        // owning face) — a small fraction, and the price of a face still being able
        // to hide its own wireframe.
        let tri_total = solid.mesh.indices.len() / 3;
        let mut wire_indices: Vec<u32> = Vec::with_capacity(solid.mesh.indices.len());
        let mut wire_ranges: Vec<(u32, u32)> = Vec::with_capacity(solid.faces.len());
        let mut covered = vec![false; tri_total];
        let mut seen: HashSet<(u32, u32), BuildEdgeHasher> = HashSet::default();
        let mut emit_triangle = |wire: &mut Vec<u32>,
                                 seen: &mut HashSet<(u32, u32), BuildEdgeHasher>,
                                 tri: usize| {
            let idx = [
                solid.mesh.indices[tri * 3],
                solid.mesh.indices[tri * 3 + 1],
                solid.mesh.indices[tri * 3 + 2],
            ];
            for k in 0..3 {
                let (a, b) = (idx[k], idx[(k + 1) % 3]);
                let key = if a < b { (a, b) } else { (b, a) };
                if seen.insert(key) {
                    wire.push(a);
                    wire.push(b);
                }
            }
        };
        for face in &solid.faces {
            let first = wire_indices.len() as u32;
            seen.clear();
            let start = (face.tri_start as usize).min(tri_total);
            let end = (start + face.tri_count as usize).min(tri_total);
            for tri in start..end {
                covered[tri] = true;
                emit_triangle(&mut wire_indices, &mut seen, tri);
            }
            wire_ranges.push((first, wire_indices.len() as u32 - first));
        }
        // Any triangle no face claims still belongs in the whole-mesh draw — the
        // fast path draws the buffer end to end. It is addressed by no face range,
        // exactly as before: the per-face path could never reach it either.
        seen.clear();
        for tri in 0..tri_total {
            if !covered[tri] {
                emit_triangle(&mut wire_indices, &mut seen, tri);
            }
        }
        let wire_index_count = wire_indices.len() as u32;
        let wire_index_buf = self
            .device
            .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some(solid.name.as_str()),
                contents: bytemuck::cast_slice(&wire_indices),
                usage: wgpu::BufferUsages::INDEX,
            });
        let (face_styles, face_style_index) = self.face_style_palette(solid, settings);
        let faces = solid
            .faces
            .iter()
            .zip(&face_style_index)
            .zip(&wire_ranges)
            .map(|((face, style), &(wire_first, wire_count))| FaceRange {
                first_index: face.tri_start * 3,
                index_count: face.tri_count * 3,
                wire_first,
                wire_count,
                style: *style,
            })
            .collect();

        let (segments, edges) = Self::edge_segments(solid);
        let occludable_edge_instances = solid
            .edges
            .iter()
            .zip(&edges)
            .find(|(edge, _)| edge.aux)
            .map_or(segments.len() as u32, |(_, range): (_, &EdgeRange)| range.first_instance);
        let edge_buf = (!segments.is_empty()).then(|| {
            self.device
                .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                    label: Some(solid.name.as_str()),
                    contents: bytemuck::cast_slice(&segments),
                    usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
                })
        });

        let points = Self::point_instances(solid);
        let point_buf = (!points.is_empty()).then(|| {
            self.device
                .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                    label: Some(solid.name.as_str()),
                    contents: bytemuck::cast_slice(&points),
                    usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
                })
        });

        let base_style = StyleBuf::new(&self.device, &self.style_layout, solid.name.as_str());
        base_style.write(
            &self.queue,
            Self::face_base_color(solid, settings),
            Self::style_params(settings),
        );

        GpuSolid {
            revision: solid.revision,
            shape: Self::shape_of(solid),
            vertex_count: vertices.len() as u32,
            face_styles,
            vertex_buf,
            index_buf,
            wire_index_buf,
            wire_index_count,
            faces,
            edge_buf,
            edges,
            edge_instances: segments.len() as u32,
            occludable_edge_instances,
            point_buf,
            point_count: points.len() as u32,
            base_style,
            boundary: None,
            // Its pose slot is given by the sync. Uploaded as it is shown,
            // so drawn through no pose; its buffers hold its shape's own
            // geometry only when it is not a pose itself.
            model_slot: 0,
            holds_shape: solid.shape_revision == 0,
        }
    }

    /// Bring the GPU scene in line with the display scene: new/changed solids
    /// upload, unchanged solids keep their buffers (R10), removed solids drop
    /// theirs. `settings_generation` invalidates per-solid style buffers only.
    pub fn sync_scene(
        &self,
        gpu: &mut GpuScene,
        scene: &RenderScene,
        settings: &RenderSettings,
        settings_generation: u64,
    ) {
        gpu.order.clear();
        let models = gpu.models.get_or_insert_with(|| ModelArena::new(&self.device, &self.model_layout, scene.solids().len() as u32));
        let mut seen: Vec<&str> = Vec::with_capacity(scene.solids().len());
        for solid in scene.solids() {
            gpu.order.push(solid.name.clone());
            seen.push(solid.name.as_str());
            let refresh_style = gpu.settings_generation != settings_generation;
            match gpu.solids.get_mut(&solid.name) {
                Some(existing) if existing.revision == solid.revision => {
                    if refresh_style {
                        self.restyle(existing, solid, settings);
                    }
                }
                // The shape whose geometry the buffers hold, shown through a
                // pose (an exploded view's step) or as it is: one matrix
                // write, nothing of the mesh rewritten.
                Some(existing)
                    if existing.holds_shape
                        && existing.shape == Self::shape_of(solid)
                        && (solid.pose.is_some() || solid.shape_revision == 0) =>
                {
                    models.set(existing.model_slot, solid.pose.as_ref());
                    existing.revision = solid.revision;
                    gpu.pose_writes += 1;
                    if refresh_style {
                        self.restyle(existing, solid, settings);
                    }
                }
                // The same shape moved rigidly (a live drag frame, its
                // restore): rewrite positions in place, rebuild nothing.
                Some(existing) if existing.shape == Self::shape_of(solid) && self.repose_solid(existing, solid) => {
                    // The buffers now hold this pose itself, drawn as it is.
                    models.set(existing.model_slot, None);
                    existing.holds_shape = false;
                    existing.revision = solid.revision;
                    gpu.reposes += 1;
                    if refresh_style {
                        self.restyle(existing, solid, settings);
                    }
                }
                _ => {
                    let mut uploaded = self.upload_solid(solid, settings);
                    match gpu.solids.get(&solid.name) {
                        Some(replaced) => uploaded.model_slot = replaced.model_slot,
                        None => uploaded.model_slot = models.alloc(),
                    }
                    models.set(uploaded.model_slot, None);
                    gpu.solids.insert(solid.name.clone(), uploaded);
                    gpu.uploads += 1;
                }
            }
        }
        let seen: std::collections::HashSet<&str> = seen.into_iter().collect();
        gpu.solids.retain(|name, solid| {
            let keep = seen.contains(name.as_str());
            if !keep {
                models.release(solid.model_slot);
            }
            keep
        });
        models.flush(&self.device, &self.queue, &self.model_layout);
        gpu.settings_generation = settings_generation;
    }

    /// The shape a display's buffers would hold: its `shape_revision` when it
    /// is a rigid move of another revision, else its own revision.
    fn shape_of(solid: &SolidDisplay) -> u64 {
        if solid.shape_revision != 0 {
            solid.shape_revision
        } else {
            solid.revision
        }
    }

    /// Rewrite a held solid's base and per-face styles for new settings.
    fn restyle(&self, existing: &GpuSolid, solid: &SolidDisplay, settings: &RenderSettings) {
        let params = Self::style_params(settings);
        existing.base_style.write(&self.queue, Self::face_base_color(solid, settings), params);
        // The per-face palette carries the same shading params, so it restyles
        // with the base or flat shading would stop applying to individually
        // coloured faces.
        let mut seen: Vec<[f32; 3]> = Vec::new();
        for face in &solid.faces {
            let Some(rgb) = face.color_override else { continue };
            if seen.contains(&rgb) {
                continue;
            }
            if let Some(style) = existing.face_styles.get(seen.len()) {
                style.write(&self.queue, Self::opaque_linear(rgb), params);
            }
            seen.push(rgb);
        }
    }

    /// Upload a scene from scratch (artifact/one-shot path).
    pub fn upload_scene(&self, scene: &RenderScene) -> GpuScene {
        self.upload_scene_with(scene, &RenderSettings::artifact())
    }

    pub fn upload_scene_with(&self, scene: &RenderScene, settings: &RenderSettings) -> GpuScene {
        let mut gpu = GpuScene::default();
        self.sync_scene(&mut gpu, scene, settings, 0);
        gpu
    }

    pub(super) fn ensure_targets(&mut self, width: u32, height: u32) {
        if let Some(targets) = &self.targets {
            if targets.width == width && targets.height == height {
                return;
            }
        }
        let size = wgpu::Extent3d {
            width,
            height,
            depth_or_array_layers: 1,
        };
        let msaa = self.device.create_texture(&wgpu::TextureDescriptor {
            label: Some("msaa color"),
            size,
            mip_level_count: 1,
            sample_count: SAMPLES,
            dimension: wgpu::TextureDimension::D2,
            format: self.format,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
            view_formats: &[],
        });
        let depth = self.device.create_texture(&wgpu::TextureDescriptor {
            label: Some("depth"),
            size,
            mip_level_count: 1,
            sample_count: SAMPLES,
            dimension: wgpu::TextureDimension::D2,
            format: DEPTH_FORMAT,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
            view_formats: &[],
        });
        self.targets = Some(CachedTargets {
            width,
            height,
            msaa_view: msaa.create_view(&Default::default()),
            depth_view: depth.create_view(&Default::default()),
        });
    }

    pub(super) fn write_global_styles(&self, settings: &RenderSettings) {
        let flat = if settings.flat_shading { 1.0 } else { 0.0 };
        let face = |c: Rgba| -> Rgba {
            [
                srgb_to_linear(c[0] as f64) as f32,
                srgb_to_linear(c[1] as f64) as f32,
                srgb_to_linear(c[2] as f64) as f32,
                c[3],
            ]
        };
        let s = &self.styles;
        let q = &self.queue;
        s.face_selected
            .write(q, face(settings.face_selected_color), [flat, 0.0, 0.0, 0.0]);
        s.face_hovered
            .write(q, face(settings.hover_color), [flat, 0.0, 0.0, 0.0]);
        // A context face picked or hovered: its selection colour, kept see-through.
        let translucent = |rgba: [f32; 4], alpha: f32| [rgba[0], rgba[1], rgba[2], alpha];
        self.context_selected.write(q, translucent(settings.face_selected_color, 0.6), [0.0; 4]);
        self.context_hovered.write(q, translucent(settings.hover_color, 0.45), [0.0; 4]);
        let edge_params = [settings.edge_width_px, EDGE_NUDGE, 0.0, 0.0];
        s.edge_base.write(q, settings.edge_color, edge_params);
        s.edge_selected.write(q, settings.edge_selected_color, edge_params);
        s.edge_hovered.write(q, settings.hover_color, edge_params);
        let mut hidden = settings.edge_color;
        hidden[3] = settings.hidden_edge_alpha;
        s.edge_hidden.write(q, hidden, edge_params);
        // An AUX edge at rest is the theme's edge at hairline WIDTH: a board's
        // copper outline is thousands of edges a fraction of a millimetre
        // apart, and at the full width the outlines of a track field crowd
        // the gaps between its tracks. (Too coarse a view does not draw them
        // at all — `frame::AUX_EDGE_MAX_WORLD_PER_PIXEL`.)
        s.edge_aux.write(q, settings.edge_color, [AUX_EDGE_WIDTH_PX, EDGE_NUDGE, 0.0, 0.0]);
        // Selected-face boundary: selected-edge color, slightly wider, extra
        // nudge so it reads as an outline on top of the fill.
        s.boundary.write(
            q,
            settings.edge_selected_color,
            [settings.edge_width_px + 1.0, EDGE_NUDGE * 1.5, 0.0, 0.0],
        );
        let point_params = [settings.vertex_size_px, 0.0, 0.0, 0.0];
        s.point_base.write(q, settings.vertex_color, point_params);
        s.point_selected
            .write(q, settings.vertex_selected_color, [settings.vertex_size_px + 1.0, 0.0, 0.0, 0.0]);
        s.point_hovered
            .write(q, settings.hover_color, [settings.vertex_size_px + 1.0, 0.0, 0.0, 0.0]);
        let axis_params = [2.0, EDGE_NUDGE, 0.0, 0.0];
        s.axis_x.write(q, [0.91, 0.30, 0.32, 1.0], axis_params);
        s.axis_y.write(q, [0.27, 0.80, 0.42, 1.0], axis_params);
        s.axis_z.write(q, [0.23, 0.51, 0.96, 1.0], axis_params);
        // Overlay-widget line width (color is per-instance from the gizmo).
        s.overlay_line.write(q, [1.0, 1.0, 1.0, 1.0], [2.5, 0.0, 0.0, 0.0]);
    }

    /// Rebuild the selected/hovered-face boundary outline buffer for one solid
    /// when the emphasis or the solid changed. The boundary of a face's
    /// triangle range = mesh edges used an odd number of times inside it.
    pub(super) fn sync_boundary(&self, gpu_solid: &mut GpuSolid, solid: &SolidDisplay, emphasis: &Emphasis) {
        if let Some(boundary) = &gpu_solid.boundary {
            if boundary.emphasis_generation == emphasis.generation
                && boundary.revision == solid.revision
            {
                return;
            }
        }
        let mut segments: Vec<EdgeInstance> = Vec::new();
        for face in &solid.faces {
            if face.tri_count == 0 {
                continue;
            }
            let state = emphasis.face_state(&solid.name, &face.name);
            if state == EmphasisState::Base {
                continue;
            }
            let mut edge_use: HashMap<(u32, u32), u32> = HashMap::new();
            let start = face.tri_start as usize;
            let end = (start + face.tri_count as usize).min(solid.mesh.indices.len() / 3);
            for tri in start..end {
                let idx = [
                    solid.mesh.indices[tri * 3],
                    solid.mesh.indices[tri * 3 + 1],
                    solid.mesh.indices[tri * 3 + 2],
                ];
                for k in 0..3 {
                    let a = idx[k];
                    let b = idx[(k + 1) % 3];
                    let key = if a < b { (a, b) } else { (b, a) };
                    *edge_use.entry(key).or_insert(0) += 1;
                }
            }
            let mut boundary: Vec<(u32, u32)> = edge_use
                .into_iter()
                .filter_map(|(key, count)| (count == 1).then_some(key))
                .collect();
            boundary.sort_unstable();
            for (a, b) in boundary {
                segments.push(EdgeInstance {
                    p0: solid.mesh.positions[a as usize],
                    p1: solid.mesh.positions[b as usize],
                });
            }
        }
        let buf = (!segments.is_empty()).then(|| {
            self.device
                .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                    label: Some("emphasis boundary"),
                    contents: bytemuck::cast_slice(&segments),
                    usage: wgpu::BufferUsages::VERTEX,
                })
        });
        gpu_solid.boundary = Some(BoundaryBuf {
            emphasis_generation: emphasis.generation,
            revision: solid.revision,
            buf,
            count: segments.len() as u32,
        });
    }
}
