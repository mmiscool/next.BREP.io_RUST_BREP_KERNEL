use super::*;

// ===========================================================================
// Import / export — the file-interchange lane (the ONE platform exception).
// STEP/IGES are text; STL/OBJ/3MF bytes are submitted to the background runner for
// RANSAC reconstruction, then return as validated STEP for IMPORT3D. Exports
// collect the CURRENT model's resident solids and serialize them.
// ===========================================================================
impl EngineState {
    /// Import an STL triangle mesh through topology-aware RANSAC recognition.
    /// Unsupported regions remain as validated facets, so every repairable
    /// source triangle reaches the resulting CAD body.
    pub fn import_stl_feature(&mut self, bytes: &[u8]) -> Result<String, String> {
        self.submit_mesh_import(crate::runner::MeshImportFormat::Stl, bytes.to_vec())
    }

    /// Import a Wavefront OBJ mesh through the same RANSAC reconstruction path.
    pub fn import_obj_feature(&mut self, text: &str) -> Result<String, String> {
        self.import_obj_bytes_feature(text.as_bytes())
    }

    /// Byte-oriented OBJ entry used by the picker so decoding also stays on the
    /// background runner with parsing and reconstruction.
    pub fn import_obj_bytes_feature(&mut self, bytes: &[u8]) -> Result<String, String> {
        self.submit_mesh_import(crate::runner::MeshImportFormat::Obj, bytes.to_vec())
    }

    /// Import a 3MF package (core specification) through the same chain. The
    /// package is unzipped and its build flattened on the runner, so a large
    /// model never decompresses on the UI thread.
    pub fn import_3mf_bytes_feature(&mut self, bytes: &[u8]) -> Result<String, String> {
        self.submit_mesh_import(crate::runner::MeshImportFormat::ThreeMf, bytes.to_vec())
    }

    fn submit_mesh_import(
        &mut self,
        format: crate::runner::MeshImportFormat,
        bytes: Vec<u8>,
    ) -> Result<String, String> {
        let id = self.submit_mesh_reconstruction(
            format, bytes, Default::default(), MeshImportDestination::Document,
        )?;
        Ok(serde_json::json!({ "meshImport": "submitted", "id": id }).to_string())
    }

    /// Reconstruct without editing history. The caller owns confirmation and
    /// can inspect the exact STEP and diagnostics returned by `take_mesh_preview`.
    pub fn reconstruct_mesh_preview(
        &mut self,
        format: crate::runner::MeshImportFormat,
        bytes: Vec<u8>,
        options: crate::runner::StlConversionOptions,
    ) -> Result<u64, String> {
        self.submit_mesh_reconstruction(format, bytes, options, MeshImportDestination::Preview)
    }

    pub fn take_mesh_preview(&mut self) -> Option<crate::runner::MeshImportReply> {
        self.mesh_preview_results.pop_front()
    }

    fn submit_mesh_reconstruction(
        &mut self,
        format: crate::runner::MeshImportFormat,
        bytes: Vec<u8>,
        options: crate::runner::StlConversionOptions,
        destination: MeshImportDestination,
    ) -> Result<u64, String> {
        if bytes.is_empty() {
            return Err("mesh import failed: file is empty".into());
        }
        let id = self.next_mesh_import_id;
        self.next_mesh_import_id = self.next_mesh_import_id.wrapping_add(1);
        self.pending_mesh_imports.insert(id, destination);
        self.runner.submit_mesh_import(crate::runner::MeshImportRequest {
            id, format, bytes, options,
        });
        self.pump();
        Ok(id)
    }

    /// Import a STEP document into the model: append an `IMPORT3D` feature whose
    /// `inputParams.stepText` is the raw ISO-10303-21 text (the exact headless
    /// source the kernel importer reads — no `fileToImport` data-URL marshaling
    /// needed), mint it a persistent-counter id, roll to it, and rebuild. Returns the
    /// build report JSON (imported bodies + any per-feature error). A non-STEP
    /// payload is refused up front so a bad upload never leaves a dead feature.
    pub fn import_step_feature(&mut self, step_text: &str) -> Result<String, String> {
        if !step_text.contains("ISO-10303-21") {
            return Err("not a STEP file (missing the ISO-10303-21 header)".into());
        }
        let id = self.next_feature_id(&crate::features::feature_short_name("IMPORT3D"));
        let feature = serde_json::json!({
            "type": "IMPORT3D",
            "inputParams": { "id": id, "stepText": step_text },
            "persistentData": {},
        });
        // The file's AP242 PMI, lifted ONCE into the document's pmi block with
        // references naming the faces / edges / vertices the feature will stamp
        // (the STEP text is never re-read for it). Read BEFORE the add so the
        // add's undo checkpoint precedes both writes: one undo removes the
        // feature and its PMI together.
        let lifted = brep_kernel::read_step_pmi(step_text, &id).unwrap_or(None);
        // Frame the imported body once the (possibly async) run lands — see
        // [`EngineState::pending_fit`]. An immediate fit here would frame the still
        // empty scene under a background runner (native thread / wasm worker).
        self.pending_fit = true;
        let report = self.add_feature(&feature.to_string());
        if let Some(lifted) = lifted {
            self.pmi_merge_imported(lifted);
        }
        report
    }

    /// Export the CURRENT model's resident solids to an ISO-10303-21 STEP
    /// document. Collects the resident handles of the rolled-to model (a warm
    /// re-run of the same prefix the display scene was built from — see
    /// [`crate::pipeline::resident_solid_handles`]) and hands them to the kernel's
    /// [`brep_kernel::export_step_handles`], so the exact NURBS topology is
    /// serialized (never the display mesh). Errs clearly when the model is empty.
    pub fn export_step_text(&mut self) -> Result<String, String> {
        self.export_step_text_named("Part")
    }

    /// [`Self::export_step_text`] with the document's own name, which becomes
    /// the root `PRODUCT`'s name (and the file's `FILE_NAME`).
    ///
    /// A document with ASSEMBLY COMPONENTS takes the STRUCTURED lane: each
    /// parts-library entry is written once as its own product, in its own local
    /// frame, and every instance becomes a `NEXT_ASSEMBLY_USAGE_OCCURRENCE`
    /// carrying its pose — nested sub-assemblies included. Without components
    /// the flat single-product writer is used, exactly as before.
    pub fn export_step_text_named(&mut self, document_name: &str) -> Result<String, String> {
        self.plugin_export_ready()?;
        // The component projection the structured lane places instances by is
        // the one the last run shipped (post-solve poses), so nothing has to be
        // synced here; the resident re-run below registers the geometry.
        let request: HistoryRequest = serde_json::from_value(self.run_request_value())
            .map_err(|e| format!("export STEP: history request: {e}"))?;
        let named = self.plugin_resident_handles(&request)?;
        // A board document's BOARD — substrate, copper, via barrels — as exact
        // solids. The viewport's board is display triangles in no registry, so
        // these are handed to the writer as values, in the ROOT product beside
        // whatever the history made (`step_solids`).
        let board = self.board_step_solids().unwrap_or_default();
        for notice in &board.notices {
            self.push_notice_as(super::NoticeSeverity::Warning, notice.clone());
        }
        if let Some(first) = board.skipped.first() {
            // The file is still written; the user is told what it lacks.
            self.push_notice(format!(
                "STEP export: {} board item(s) could not be made as solids and are missing from the file — first: {first}",
                board.skipped.len()
            ));
        }
        if named.is_empty() && board.bodies.is_empty() {
            return Err("nothing to export: the model has no solids".into());
        }
        // The document's PMI rides along as AP242 semantic PMI + saved views,
        // resolved against the same resident solids the file is written from
        // (the warm re-run above left the tail's report on this thread).
        let report = request
            .pmi
            .as_ref()
            .map(|_| {
                let _trace = crate::run_trace::span("export_pmi");
                self.execute_plugin_history(&request).pmi
            })
            .flatten();
        let annotation_errors = super::plugins::plugin_annotation_errors(report.as_ref());
        if !annotation_errors.is_empty() {
            return Err(format!("plugin annotation export replay failed: {}", annotation_errors.join("; ")));
        }
        let pmi = match (request.pmi.as_ref(), report.as_ref()) {
            (Some(state), Some(report)) => Some(brep_kernel::StepPmi { state, report }),
            _ => None,
        };
        let mut colors = self.step_export_colors(&named);
        for (name, rgb) in &board.colors {
            // A colour the user stored under a board body's name still wins.
            if self.metadata.attribute(name, brep_kernel::COLOR_METADATA_KEY).is_none() {
                colors.set(name, *rgb);
            }
        }
        // The resident solids by value — read on THIS thread, where the re-run
        // above registered them — whenever the handle-taking writers cannot
        // take the whole file: there is a board to add, or a product id to fix.
        let resident = |named: &[(String, u32)]| -> Result<Vec<(String, brep_kernel::BrepSolid)>, String> {
            named
                .iter()
                .map(|(name, handle)| Ok((name.clone(), brep_kernel::registered_solid_clone(*handle)?)))
                .collect()
        };
        if self.assembly_components.is_empty() && !board.bodies.is_empty() {
            let mut solids = resident(&named)?;
            solids.extend(board.bodies);
            let borrowed: Vec<(String, &brep_kernel::BrepSolid)> =
                solids.iter().map(|(name, solid)| (name.clone(), solid)).collect();
            return brep_kernel::export_step_report_named(
                &borrowed,
                document_name,
                "MM",
                "",
                pmi.as_ref(),
                Some(&colors),
            )
            .map(|report| report.text);
        }
        if self.assembly_components.is_empty() {
            return brep_kernel::export_step_named_handles(
                &named,
                document_name,
                "MM",
                "",
                pmi.as_ref(),
                Some(&colors),
            )
            .map(|report| report.text);
        }
        let components: Vec<(String, String, brep_kernel::Mat4)> = self
            .assembly_components
            .iter()
            .map(|record| {
                (
                    record.id.clone(),
                    record.part_name.clone(),
                    record.transform,
                )
            })
            .collect();
        let resident = resident(&named)?;
        let mut assembly = self.with_plugin_provider(||
            brep_kernel::assembly_export_tree(document_name, resident, &components))?;
        // The board is a product of its own, placed in the root at identity,
        // so a receiving system's tree lists it by name (`PCB`) beside the
        // parts. As the root's own geometry it read as an unnamed compound
        // (OpenCascade: `=>[0:1:1:6]`). Its colours are keyed again under the
        // occurrence's namespace, which is how the writer resolves them.
        if !board.bodies.is_empty() {
            for (name, rgb) in &board.colors {
                if self.metadata.attribute(name, brep_kernel::COLOR_METADATA_KEY).is_none() {
                    colors.set(&format!("{BOARD_OCCURRENCE}:{name}"), *rgb);
                }
            }
            for (name, rgb) in self.persisted_export_colors() {
                if name.starts_with(super::board_geometry::BOARD_SOLID_PREFIX) {
                    colors.set(&format!("{BOARD_OCCURRENCE}:{name}"), rgb);
                }
            }
            assembly.products.push(brep_kernel::StepExportProduct {
                name: BOARD_OCCURRENCE.to_string(),
                id: BOARD_OCCURRENCE.to_string(),
                bodies: board.bodies,
                pmi: None,
            });
            assembly.occurrences.push(brep_kernel::StepExportOccurrence {
                designator: BOARD_OCCURRENCE.to_string(),
                parent: 0,
                child: assembly.products.len() - 1,
                placement: [
                    1., 0., 0., 0., //
                    0., 1., 0., 0., //
                    0., 0., 1., 0., //
                    0., 0., 0., 1.,
                ],
            });
        }
        for product in &mut assembly.products {
            product.id = portable_product_id(&product.id, &product.name);
        }
        if self.history.pcb_block().is_some() && pmi.is_none() {
            self.label_occurrences_by_reference(&mut assembly, &mut colors, &named);
        }
        brep_kernel::export_step_assembly_report(&assembly, "MM", "", pmi.as_ref(), Some(&colors))
            .map(|report| report.text)
    }

    /// A BOARD document's placed parts, named in the STEP tree by their
    /// REFERENCE DESIGNATOR (`U1`, `R3`) instead of the component feature id
    /// (`ACOMP1`) — the name the engineer on the other end knows the part by.
    ///
    /// The occurrence label is also the namespace the writer resolves the
    /// document's names through (`ACOMP1:IMPORT3D1_Face_0` finds its face under
    /// the `ACOMP1:` occurrence), so every colour keyed under a relabelled
    /// component is keyed again under its new label, or the KiCad body colours
    /// would drop out of the file. PMI resolves through the same namespace,
    /// which is why the caller only relabels a document that carries none.
    ///
    /// A reference is used only when it is unambiguous: non-empty, free of the
    /// `:` namespace separator, carried by exactly one component, and neither
    /// some other component's id nor the board's own label
    /// ([`BOARD_OCCURRENCE`]). Anything else keeps its feature id.
    fn label_occurrences_by_reference(
        &self,
        assembly: &mut brep_kernel::StepAssemblyExport,
        colors: &mut brep_kernel::StepColors,
        named: &[(String, u32)],
    ) {
        let mut references: std::collections::BTreeMap<String, String> = Default::default();
        let mut seen: std::collections::BTreeMap<String, usize> = Default::default();
        let mut ids: std::collections::BTreeSet<String> = std::collections::BTreeSet::new();
        for feature in self.history.features() {
            let is_component = feature["type"]
                .as_str()
                .is_some_and(super::components::is_acomp_feature_type);
            let params = &feature["inputParams"];
            let Some(id) = params["id"].as_str().filter(|_| is_component) else {
                continue;
            };
            ids.insert(id.to_string());
            let reference = params["bom"]["Reference_Designator"].as_str().unwrap_or("").trim();
            if !reference.is_empty() && !reference.contains(':') {
                references.insert(id.to_string(), reference.to_string());
                *seen.entry(reference.to_string()).or_default() += 1;
            }
        }
        references.retain(|_, reference| {
            seen[reference.as_str()] == 1 && !ids.contains(reference.as_str()) && reference != BOARD_OCCURRENCE
        });
        if references.is_empty() {
            return;
        }
        for occurrence in assembly.occurrences.iter_mut().filter(|o| o.parent == 0) {
            if let Some(reference) = references.get(&occurrence.designator) {
                occurrence.designator = reference.clone();
            }
        }
        // The same colours `step_export_colors` keyed, keyed again under the
        // new label: the persisted ones, and the name-hash look's.
        let hashed = self.settings.face_color_mode == crate::style::FaceColorMode::HashedBySolid;
        let mut keyed: Vec<(String, [u8; 3])> = self.persisted_export_colors();
        if hashed {
            for (name, _) in named {
                if self.metadata.attribute(name, brep_kernel::COLOR_METADATA_KEY).is_none() {
                    keyed.push((name.clone(), srgb_bytes(crate::color::solid_color_srgb(name))));
                }
            }
        }
        for (name, rgb) in keyed {
            let Some((id, rest)) = name.split_once(':') else {
                continue;
            };
            if let Some(reference) = references.get(id) {
                colors.set(&format!("{reference}:{rest}"), rgb);
            }
        }
    }

    /// The body/face colours the STEP writer should emit as presentation styles.
    ///
    /// The METADATA STORE is the authority, exactly as it is for the viewport
    /// ([`EngineState::sync_colors_from_metadata`]): every `color` attribute the
    /// document persists — a STEP import's stamped colours, a colour picked in
    /// the Info window, a colour restored from a saved file — becomes a style on
    /// the body or face of that name. Spellings are read through
    /// [`crate::style::parse_css_hex`], the one parser the display uses, so a
    /// value the viewport shows is a value the file carries.
    ///
    /// `override_model_colors` is deliberately NOT honoured: it is a display
    /// switch whose own contract is that the store is never touched, so a
    /// document exported with the box ticked still carries all of its colours.
    ///
    /// The NAME-DERIVED fallback is the second half, and it is narrow on
    /// purpose: a body with no persisted colour gets the stable name-hash
    /// colour ONLY when the viewport is itself colouring by name hash
    /// ([`crate::style::FaceColorMode::HashedBySolid`] — the artifact look, not
    /// the default). In the default `Uniform` mode an uncoloured body gets no
    /// style at all, so an export from a document nobody coloured is
    /// byte-for-byte the file this writer produced before colour existed rather
    /// than a model painted in colours the user never saw.
    ///
    /// Faces are never name-derived: the renderer hashes per SOLID, so there is
    /// no per-face colour to match.
    fn step_export_colors(&self, named: &[(String, u32)]) -> brep_kernel::StepColors {
        let mut colors = brep_kernel::StepColors::new();
        for (name, rgb) in self.persisted_export_colors() {
            colors.set(&name, rgb);
        }
        if self.settings.face_color_mode == crate::style::FaceColorMode::HashedBySolid {
            for (name, _) in named {
                if self.metadata.attribute(name, brep_kernel::COLOR_METADATA_KEY).is_none() {
                    colors.set(name, srgb_bytes(crate::color::solid_color_srgb(name)));
                }
            }
        }
        colors
    }

    /// Resident handle of the part's target sheet-metal body for a flat-pattern
    /// export. Enumerates the current resident solids (a warm re-run of the same
    /// prefix the display scene was built from, like the STEP lane) and keeps the
    /// ones carrying a sheet-metal tree; uses the SELECTED sheet-metal body if the
    /// selection names exactly one, else the SOLE sheet-metal body (the same
    /// auto-target SM.CUTOUT uses). Errs with the exact `"no sheet-metal body in
    /// the part"` when there is none, and loudly when several are ambiguous.
    fn flat_pattern_target_handle(&self) -> Result<u32, String> {
        self.plugin_export_ready()?;
        let request: HistoryRequest = serde_json::from_value(self.run_request_value())
            .map_err(|e| format!("export flat pattern: history request: {e}"))?;
        let sheet_metal: Vec<(String, u32)> = self.plugin_resident_handles(&request)?
            .into_iter()
            .filter(|(_, handle)| brep_kernel::is_sheet_metal_handle(*handle))
            .collect();
        if sheet_metal.is_empty() {
            return Err("no sheet-metal body in the part".into());
        }
        // Prefer a selected sheet-metal body when the selection names exactly one.
        let selected: Vec<u32> = sheet_metal
            .iter()
            .filter(|(name, _)| self.emphasis.selected_solids.contains(name))
            .map(|(_, handle)| *handle)
            .collect();
        if let [handle] = selected.as_slice() {
            return Ok(*handle);
        }
        match sheet_metal.as_slice() {
            [(_, handle)] => Ok(*handle),
            _ => Err(
                "several sheet-metal bodies in the part — select the one to export".into(),
            ),
        }
    }

    /// Export the part's sheet-metal FLAT PATTERN (the unfold) as a DXF (R12
    /// ASCII) 2D vector document. Runs the unfold TRANSIENTLY off the target
    /// body's resident tree — no feature is added and history is not mutated. Errs
    /// (`"no sheet-metal body in the part"`) when the part carries no sheet metal.
    pub fn export_flat_pattern_dxf(&self) -> Result<String, String> {
        brep_kernel::flat_pattern_dxf(self.flat_pattern_target_handle()?)
    }

    /// Export the part's sheet-metal flat pattern as an SVG — the DXF sibling of
    /// [`Self::export_flat_pattern_dxf`].
    pub fn export_flat_pattern_svg(&self) -> Result<String, String> {
        brep_kernel::flat_pattern_svg(self.flat_pattern_target_handle()?)
    }

    /// Import an IGES document into the model: append an `IMPORT3D` feature whose
    /// `inputParams.igesText` is the raw IGES text (the kernel importer reads it
    /// via [`brep_kernel::import_iges`]), mint an id, roll to it, and rebuild.
    /// Refuses a non-IGES payload up front so a bad upload never leaves a dead
    /// feature.
    pub fn import_iges_feature(&mut self, iges_text: &str) -> Result<String, String> {
        if iges_text.contains("ISO-10303-21") {
            return Err("not an IGES file (this looks like a STEP document)".into());
        }
        // IGES records carry an S/G/D/P/T section letter in column 73.
        let looks_like_iges = iges_text.lines().any(|line| {
            matches!(line.chars().nth(72), Some('S' | 'G' | 'D' | 'P' | 'T'))
        });
        if !looks_like_iges {
            return Err("not an IGES file (no S/G/D/P/T section records found)".into());
        }
        let id = self.next_feature_id(&crate::features::feature_short_name("IMPORT3D"));
        let feature = serde_json::json!({
            "type": "IMPORT3D",
            "inputParams": { "id": id, "igesText": iges_text },
            "persistentData": {},
        });
        // Frame the imported body once the (possibly async) run lands — see
        // [`EngineState::pending_fit`] (mirrors the STEP lane above).
        self.pending_fit = true;
        self.add_feature(&feature.to_string())
    }

    /// Export the CURRENT model's resident solids to an IGES 5.3 document of
    /// trimmed NURBS surfaces — the IGES analogue of [`Self::export_step_text`],
    /// handing the resident handles to [`brep_kernel::export_iges_handles`].
    pub fn export_iges_text(&self) -> Result<String, String> {
        self.plugin_export_ready()?;
        let request: HistoryRequest = serde_json::from_value(self.run_request_value())
            .map_err(|e| format!("export IGES: history request: {e}"))?;
        let handles: Vec<u32> = self.plugin_resident_handles(&request)?
            .into_iter()
            .map(|(_, handle)| handle)
            .collect();
        if handles.is_empty() {
            return Err("nothing to export: the model has no solids".into());
        }
        brep_kernel::export_iges_handles(&handles, "Part", "MM", "")
    }

    /// Export the CURRENT display scene to an ASCII STL string (one `solid` with a
    /// per-triangle geometric normal for every mesh triangle of every displayed
    /// solid). STL is a triangle-soup format with no multi-body concept, so all
    /// solids fold into a single `solid brep … endsolid brep`. String-shaped so it
    /// crosses the same string `ModelStore` seam the STEP lane uses. Errs when the
    /// scene has no triangles.
    pub fn export_stl_text(&self) -> Result<String, String> {
        self.plugin_export_ready()?;
        let mut out = String::from("solid brep\n");
        let mut triangles = 0usize;
        for solid in self.scene.solids() {
            let positions = &solid.mesh.positions;
            for tri in solid.mesh.indices.chunks_exact(3) {
                let a = positions[tri[0] as usize];
                let b = positions[tri[1] as usize];
                let c = positions[tri[2] as usize];
                let normal = triangle_normal(a, b, c);
                out.push_str(&format!(
                    "  facet normal {} {} {}\n    outer loop\n",
                    normal[0], normal[1], normal[2]
                ));
                for v in [a, b, c] {
                    out.push_str(&format!("      vertex {} {} {}\n", v[0], v[1], v[2]));
                }
                out.push_str("    endloop\n  endfacet\n");
                triangles += 1;
            }
        }
        out.push_str("endsolid brep\n");
        if triangles == 0 {
            return Err("nothing to export: the scene has no triangles".into());
        }
        Ok(out)
    }

    /// Export the CURRENT display scene as Wavefront OBJ text — the same
    /// triangles [`Self::export_stl_text`] writes, folded into ONE object `brep`
    /// with the display normals kept per vertex, through the kernel's
    /// [`brep_kernel::write_obj`]. Millimetres, like every other export lane.
    /// Errs when the scene has no triangles.
    pub fn export_obj_text(&self) -> Result<String, String> {
        self.plugin_export_ready()?;
        let mut mesh = brep_kernel::Mesh::default();
        for solid in self.scene.solids() {
            let base = (mesh.positions.len() / 3) as u32;
            for (position, normal) in solid.mesh.positions.iter().zip(&solid.mesh.normals) {
                mesh.positions.extend(position.iter().map(|value| f64::from(*value)));
                mesh.normals.extend(normal.iter().map(|value| f64::from(*value)));
            }
            mesh.indices.extend(solid.mesh.indices.iter().map(|index| base + index));
        }
        if mesh.indices.is_empty() {
            return Err("nothing to export: the scene has no triangles".into());
        }
        brep_kernel::write_obj(&mesh, "brep")
    }

    /// Every `color` attribute the document persists, as 8-bit sRGB keyed by
    /// scene name. The METADATA STORE is the authority for BOTH export lanes —
    /// the STEP writer's styles ([`Self::step_export_colors`]) and the GLB
    /// writer's materials ([`Self::export_glb_bytes`]) — so a colour the Info
    /// window's Metadata tab shows is the colour either file carries. Spellings
    /// are read through [`crate::style::parse_css_hex`], the one parser the
    /// display itself uses.
    fn persisted_export_colors(&self) -> Vec<(String, [u8; 3])> {
        self.metadata
            .all()
            .iter()
            .filter_map(|(name, record)| {
                let value = record.get(brep_kernel::COLOR_METADATA_KEY)?;
                let rgb = crate::style::parse_css_hex(value)?;
                Some((name.clone(), srgb_bytes(rgb.map(f64::from))))
            })
            .collect()
    }

    /// The colour the VIEWPORT is showing a body with no persisted colour of
    /// its own: the uniform `face_color` setting, or the stable name-hash in the
    /// artifact look. This is the one place the GLB lane parts company with the
    /// STEP lane, which leaves such a body unstyled — glTF has no "no style"
    /// state, so a primitive with no material renders in the format's own
    /// default (white, fully metallic), a colour nobody chose.
    fn display_body_color(&self, name: &str) -> [u8; 3] {
        match self.settings.face_color_mode {
            crate::style::FaceColorMode::HashedBySolid => {
                srgb_bytes(crate::color::solid_color_srgb(name))
            }
            crate::style::FaceColorMode::Uniform => {
                let face = self.settings.face_color;
                srgb_bytes([f64::from(face[0]), f64::from(face[1]), f64::from(face[2])])
            }
        }
    }

    /// Export the CURRENT display scene as a binary glTF 2.0 (`.glb`) document —
    /// the same triangles [`Self::export_stl_text`] and [`Self::export_obj_text`]
    /// write, through the kernel's [`brep_kernel::write_glb`]. Unlike those two,
    /// glTF HAS a multi-body concept, so each scene solid becomes its own named
    /// mesh instead of folding into one object.
    ///
    /// BINARY, not text: the bytes cross the store's byte-shaped interchange
    /// (`export_file_named_bytes`), because base64 through the text lane would
    /// inflate the file by a third for nothing.
    ///
    /// The axis convention (+Y up, which is both glTF's and this application's,
    /// so the identity) and the millimetre-to-metre unit both ride on the root
    /// NODE's transform — see the kernel module's doc. COLOURS are the metadata
    /// store's, the same authority the STEP lane reads, with a face's beating its
    /// body's; a body the store says nothing about takes the colour the viewport
    /// shows it in ([`Self::display_body_color`]). `override_model_colors` is not
    /// honoured, exactly as it is not by the STEP lane: it is a display switch
    /// whose contract is that the store is never touched.
    ///
    /// Errs when the scene has no triangles.
    pub fn export_glb_bytes(&self, document_name: &str) -> Result<Vec<u8>, String> {
        self.plugin_export_ready()?;
        let colors: std::collections::BTreeMap<String, [u8; 3]> =
            self.persisted_export_colors().into_iter().collect();
        let solids: Vec<brep_kernel::GlbSolid> = self
            .scene
            .solids()
            .iter()
            .map(|solid| {
                let mut mesh = brep_kernel::Mesh::default();
                for (position, normal) in solid.mesh.positions.iter().zip(&solid.mesh.normals) {
                    mesh.positions.extend(position.iter().map(|value| f64::from(*value)));
                    mesh.normals.extend(normal.iter().map(|value| f64::from(*value)));
                }
                mesh.indices = solid.mesh.indices.clone();
                // The kernel writer groups by the per-TRIANGLE face id, which is
                // an index into this solid's own `faces` — the same numbering
                // `face_colors` is keyed by below.
                mesh.face_ids = solid.mesh.face_ids.clone();
                let face_colors = solid
                    .faces
                    .iter()
                    .enumerate()
                    .filter_map(|(index, face)| {
                        colors.get(&face.name).map(|rgb| (index as u32, *rgb))
                    })
                    .collect();
                brep_kernel::GlbSolid {
                    name: solid.name.clone(),
                    mesh,
                    color: Some(
                        colors
                            .get(&solid.name)
                            .copied()
                            .unwrap_or_else(|| self.display_body_color(&solid.name)),
                    ),
                    face_colors,
                }
            })
            .collect();
        brep_kernel::write_glb(
            &solids,
            document_name,
            brep_kernel::GlbUpAxis::YUp,
            METRES_PER_MILLIMETRE,
        )
        .map(|(bytes, _report)| bytes)
    }
    /// The solids a 3D PDF shows ([`crate::sheets::pdf3d::Body`]): every
    /// visible, non-sketch solid with triangles, UN-POSED (an explode the
    /// active PMI view applies to the viewport is that view's, not the
    /// model's), coloured by the GLB lane's rule — a face's persisted colour,
    /// else its body's, else the colour the viewport shows the body in — with
    /// its real edges (not the auxiliary or centre-line display edges).
    pub fn pdf3d_bodies(&self) -> Vec<crate::sheets::pdf3d::Body> {
        let colors: std::collections::BTreeMap<String, [u8; 3]> =
            self.persisted_export_colors().into_iter().collect();
        self.scene
            .solids()
            .iter()
            .filter(|solid| solid.visible && !solid.is_sketch && solid.mesh.indices.len() >= 3)
            .map(|shown| {
                let solid = self.pmi_explode_originals.get(&shown.name).unwrap_or(shown);
                let body = colors.get(&solid.name).copied().unwrap_or_else(|| self.display_body_color(&solid.name));
                let face_colors: Vec<[u8; 3]> = solid
                    .faces
                    .iter()
                    .map(|face| colors.get(&face.name).copied().unwrap_or(body))
                    .collect();
                crate::sheets::pdf3d::Body {
                    name: solid.name.clone(),
                    positions: solid.mesh.positions.clone(),
                    normals: solid.mesh.normals.clone(),
                    indices: solid.mesh.indices.clone(),
                    triangle_colors: solid
                        .mesh
                        .face_ids
                        .iter()
                        .map(|face| face_colors.get(*face as usize).copied().unwrap_or(body))
                        .collect(),
                    edges: solid
                        .edges
                        .iter()
                        .filter(|edge| !edge.aux && !edge.centerline && edge.polyline.len() >= 2)
                        .map(|edge| edge.polyline.clone())
                        .collect(),
                }
            })
            .collect()
    }
}

/// The model's unit expressed in glTF's. Every export lane writes millimetres
/// (the STEP writer's `MM`) and glTF's own linear unit is the METRE, so the GLB
/// root node carries this as its scale rather than the vertices being rewritten.
const METRES_PER_MILLIMETRE: f64 = 0.001;

/// Unit (or zero, for a degenerate triangle) geometric normal of triangle
/// `(a, b, c)` — the per-facet normal an ASCII STL record carries.
fn triangle_normal(a: [f32; 3], b: [f32; 3], c: [f32; 3]) -> [f32; 3] {
    let u = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    let v = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
    let n = [
        u[1] * v[2] - u[2] * v[1],
        u[2] * v[0] - u[0] * v[2],
        u[0] * v[1] - u[1] * v[0],
    ];
    let len = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
    if len > 0.0 {
        [n[0] / len, n[1] / len, n[2] / len]
    } else {
        [0.0, 0.0, 0.0]
    }
}

// ===========================================================================
// STRUCTURED STEP import — the assembly lane.
//
// The flat lane above (`import_step_feature`) appends ONE IMPORT3D holding the
// raw Part-21 text and lets the kernel bake every occurrence's world transform
// into its own body: N bodies, no parts, no tree. This lane keeps the structure
// instead — each unique geometry-bearing PRODUCT_DEFINITION becomes ONE
// parts-library entry holding a NATIVE payload (`nativeBrep`, no STEP text
// anywhere past this door), and each occurrence of it becomes an ACOMP instance
// carrying the composed world pose. Six bolts are then one entry × six
// instances, which is what makes the BOM, the structure tree, per-component
// selection and constraints work on imported geometry.
//
// # FLAT or NESTED — the user's choice, both correct
//
// [`StepAssemblyImport::nested`] picks between two shapes of the same
// geometry:
//
// - **Flat** flattens the occurrence tree to its geometry-bearing leaves: one
//   ACOMP per leaf occurrence, each carrying the COMPOSED world pose. Every
//   part is stored once for the whole document.
// - **Nested** keeps the tree: each assembly-node product becomes a part
//   document that itself carries `{partsLibrary, features: [ACOMP…,
//   IMPORT3D…]}`, built bottom-up by the same recursive builder, and the
//   parent gets ONE ACOMP per sub-assembly occurrence.
//
// Neither is the deprecated one. Nested shows the real tree; flat is the right
// answer for a deep or pathological file, and it stores a part reused at two
// levels ONCE, where nesting stores it once PER LEVEL. For a depth-1 tree the
// two lanes produce byte-identical documents — the cheapest correctness check
// there is, and `nested_matches_flat_for_a_depth_one_tree` asserts exactly it.
//
// # PROBE then CONSUME — because the parse is the expensive half
//
// The app must know the counts BEFORE it can offer the choice ("7 parts, 23
// instances — import as assembly or as bodies?"), and re-reading multi-MB
// Part-21 text after the user clicks would pay the file's single most expensive
// cost twice. So [`EngineState::probe_step_assembly`] performs the ONE parse and
// stashes the [`brep_kernel::StepAssembly`] in
// [`EngineState::pending_step_assembly`];
// [`EngineState::import_probed_step_assembly`] TAKES it. Cancel
// ([`EngineState::discard_probed_step_assembly`]), a second probe, and a
// document switch all drop it, so a user who cancels three imports is holding
// zero parsed assemblies — a real consideration, since the stash keeps every
// product's solids resident for as long as the dialog is open.
//
// # ONE rebuild for the whole import
//
// `add_feature` re-runs the entire history per call, so appending N instances
// through it is O(N²). This lane appends them all through
// [`EngineState::add_features`] — one push batch, one rebuild, one undo step.
// (Not `set_history_json`: that is the document-SWITCH path, which clears the
// kernel history cache and resets the runner's delta baseline.)
//
// # Fallback, never a silent zero
//
// No structure at all, or every geometry-bearing product failing to encode, both
// end at today's flat lane. "A successful import that produces zero components"
// is a failure wearing a result's clothes, so the zero-component case is an
// `Err` for the dialog-driven entry point (the app owns the file text and re-runs
// the flat import) and an automatic fall-back for the text-taking convenience.
// ===========================================================================

/// What the import dialog needs to describe a STEP file's structure — counts
/// only, so the probe can answer without building anything.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StepAssemblyProbe {
    /// Unique geometry-bearing products → parts-library entries. The floor, not
    /// the final count: a non-rigid occurrence bakes its own extra entry (§3.4).
    pub parts: usize,
    /// Geometry-bearing occurrences → ACOMP instance features.
    pub instances: usize,
    /// Longest root→node chain of occurrences. `1` is a flat assembly; `> 1`
    /// means sub-assemblies exist, so [`StepAssemblyImport::nested`] changes
    /// the shape of the result and the dialog's choice is worth offering.
    pub nested_depth: usize,
}

/// The choices the import dialog collects.
#[derive(Debug, Clone, Copy, Default)]
pub struct StepAssemblyImport {
    /// Build nested rigid sub-assembly documents instead of flattening the
    /// tree to its leaf occurrences.
    ///
    /// `false` (the `Default`) is the flat lane, byte-for-byte unchanged. On a
    /// depth-1 tree the two produce the same document, so this flag only ever
    /// matters for a file that really has sub-assemblies.
    pub nested: bool,
}

/// What an import did — the numbers the status line and notice report.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct StepAssemblyReport {
    /// Parts-library entries this import added or reused — the entries of the
    /// USER'S document. On a nested import that is the top level only: a
    /// sub-assembly's own entries live in ITS document's library, which the
    /// parent never sees.
    pub parts: usize,
    /// ACOMP instance features appended to the user's document. Nested: one per
    /// ROOT-level occurrence (a sub-assembly is one component), not one per
    /// leaf body.
    pub instances: usize,
    /// Occurrences whose non-rigid factor was baked into a distinct part
    /// (§3.4), summed over every level a nested import built.
    pub baked_nonrigid: usize,
    /// Products (or baked non-rigid variants of one) that did not encode to a
    /// payload — skipped and counted, never fatal: the importer's
    /// graceful-degradation contract, carried up to this altitude. Summed over
    /// every level a nested import built.
    pub failed_products: usize,
    /// The first thing that went wrong, from the kernel's body-build errors or
    /// this lane's own encode failures.
    pub first_error: Option<String>,
    /// The structured lane did not run: the file carries no usable structure, or
    /// nothing in it encoded, so the bodies were imported through the flat lane
    /// exactly as before. Only ever `true` from [`EngineState::import_step_assembly`],
    /// which holds the text; the dialog-driven entry point returns `Err` instead
    /// and lets its caller re-run the flat import it already has the text for.
    pub flat_fallback: bool,
}

/// The outcome of consuming a parsed assembly, before it is shaped into either
/// an `Err` (dialog lane) or a flat fallback (text lane) — so neither has to
/// recognise "nothing imported" by matching an error string.
enum Consumed {
    Imported(StepAssemblyReport),
    /// Every geometry-bearing product failed to encode: no components, so this
    /// is not an import.
    NoComponents {
        failed_products: usize,
        first_error: Option<String>,
    },
}

/// A row-major 4×4 affine, the shape `StepOccurrence::placement` and
/// `AffineTransform` both use.
type Mat4 = [f64; 16];

/// One node of the composed occurrence tree — a product at a world pose.
struct PlacedProduct {
    /// Index into `StepAssembly::products`.
    product: usize,
    /// Composed child-local → world transform.
    world: Mat4,
    /// Occurrence edges between a root and this node (`0` at a root).
    depth: usize,
    /// Every edge on the path here was rigid, so `world` IS a component pose.
    /// False means the non-rigid factor must be baked into the part (§3.4).
    rigid_path: bool,
    /// The STEP occurrence edges walked to get here, root first — the chain a
    /// lifted OCCURRENCE-scoped annotation names its geometry through
    /// (`brep_kernel::rewrite_occurrence_refs`). Empty at a root.
    chain: Vec<usize>,
}

/// A parts-library entry this import needs: a product, plus the bits of the
/// non-rigid factor baked into it (all-zero linear block ⇒ none). Two
/// occurrences of one product under DIFFERENT non-rigid factors are different
/// parts — never a wrong-handed reuse.
type PartKey = (usize, [u64; 9]);

/// The `PartKey` factor slot for a plain rigid instance.
const NO_FACTOR: [u64; 9] = [0; 9];

impl EngineState {
    /// Read a STEP file's product structure — THE parse of a structured import.
    /// Stashes the parsed assembly (with every product's solids) for
    /// [`Self::import_probed_step_assembly`] and returns the dialog's counts.
    ///
    /// `Ok(None)` = no usable structure (no NAUO edges, or none reaching built
    /// geometry): the caller imports through the flat
    /// [`Self::import_step_feature`] lane with the text it already holds, which
    /// is byte-for-byte today's behaviour. `Err` only for text that is not a
    /// Part 21 file at all — a BROKEN assembly degrades, it does not fail.
    ///
    /// Replaces any previously stashed assembly on EVERY outcome, `Ok(None)`
    /// included: a stale stash surviving a probe of a different file is how a
    /// consume silently imports the wrong one.
    pub fn probe_step_assembly(
        &mut self,
        step_text: &str,
    ) -> Result<Option<StepAssemblyProbe>, String> {
        let id = self.submit_step_probe(step_text);
        // Inline answers inside `submit`'s own pump; a background runner has
        // not answered yet and this call must not pretend it has.
        match self.take_step_probe() {
            Some((answered, outcome)) if answered == id => match outcome {
                super::StepProbeOutcome::Structure(probe) => Ok(Some(probe)),
                super::StepProbeOutcome::Flat => Ok(None),
                super::StepProbeOutcome::Failed(error) => Err(error),
            },
            _ => Err(
                "the STEP probe is still running on the background runner — use \
                 submit_step_probe / take_step_probe"
                    .into(),
            ),
        }
    }

    /// SUBMIT a STEP text to be probed for product structure on the runner
    /// (the parse builds every product's bodies: seconds for a real assembly,
    /// which is why it leaves the UI thread). The answer arrives through
    /// [`Self::take_step_probe`] under the returned id, after a later `pump`;
    /// a found structure is stashed for [`Self::import_probed_step_assembly`].
    /// Any earlier stash is dropped now — the probe REPLACES it on every
    /// outcome, so a stale parse can never be consumed for the wrong file.
    ///
    /// The structure test the parse would make is "any NEXT_ASSEMBLY_USAGE_
    /// OCCURRENCE entity" (`assembly_edges`), so a text with none cannot have
    /// structure and is answered `Flat` without a trip to the runner: a part
    /// file — the common upload — no longer pays a full parse only to be told
    /// to take the flat lane, where the worker parses it anyway. (The text
    /// test is a superset of the entity test: a stray mention in a comment
    /// merely runs the parse.)
    pub fn submit_step_probe(&mut self, step_text: &str) -> u64 {
        self.pending_step_assembly = None;
        let id = self.next_step_probe_id;
        self.next_step_probe_id = self.next_step_probe_id.wrapping_add(1);
        if !step_text.contains("NEXT_ASSEMBLY_USAGE_OCCURRENCE") {
            self.step_probe_results
                .push_back((id, super::StepProbeOutcome::Flat));
            return id;
        }
        self.pending_step_probes.insert(id);
        self.runner.submit_step_probe(crate::runner::StepProbeRequest {
            id,
            text: step_text.to_string(),
        });
        self.pump();
        id
    }

    /// The oldest answered probe, if any: its submission id and what it found.
    pub fn take_step_probe(&mut self) -> Option<(u64, super::StepProbeOutcome)> {
        self.step_probe_results.pop_front()
    }

    /// Whether a submitted probe has not been answered yet — the app keeps the
    /// frame loop alive (and the panel its "reading…" status) while it is.
    pub fn step_probes_pending(&self) -> bool {
        !self.pending_step_probes.is_empty()
    }

    /// Import the assembly [`Self::probe_step_assembly`] stashed: one
    /// parts-library entry per unique product, one ACOMP instance per
    /// occurrence, ONE rebuild. TAKES the stash, so a double-import is an error
    /// rather than a double-insert.
    ///
    /// `doc_name` names products the file left unnamed (`{doc_name}-part-{id}`).
    /// `opts.nested` chooses between the flat and nested shapes — see
    /// [`StepAssemblyImport::nested`]. Errs when nothing is stashed, and when
    /// every product failed to encode — the latter being the caller's cue to
    /// re-run the flat import with the file text it holds.
    /// `sink` receives every unique part document so the app can write it to
    /// the model store and hand back a real `sourceKey`; pass [`EmbeddedOnly`]
    /// to keep the parts embedded (what a caller with no store does).
    pub fn import_probed_step_assembly(
        &mut self,
        doc_name: &str,
        opts: StepAssemblyImport,
        sink: &mut dyn PartSink,
    ) -> Result<StepAssemblyReport, String> {
        let assembly = self.pending_step_assembly.take().ok_or_else(|| {
            "import STEP assembly: nothing probed (call probe_step_assembly first)".to_string()
        })?;
        match self.consume_step_assembly(assembly, doc_name, opts.nested, sink) {
            Consumed::Imported(report) => Ok(report),
            Consumed::NoComponents { first_error, .. } => Err(format!(
                "import STEP assembly: no part of the assembly could be built{}",
                first_error
                    .map(|error| format!(" ({error})"))
                    .unwrap_or_default()
            )),
        }
    }

    /// Drop a probed assembly and the solids it holds resident — the dialog's
    /// Cancel. Idempotent.
    pub fn discard_probed_step_assembly(&mut self) {
        self.pending_step_assembly = None;
    }

    /// Probe + consume in one call, falling back to the flat lane by itself —
    /// the HEADLESS/test entry point. The app uses the probe/consume pair
    /// instead, because it has a dialog between the two halves.
    ///
    /// Still exactly one parse: this is `probe_step_assembly` followed by the
    /// consume of what it stashed.
    ///
    /// Parts stay EMBEDDED here ([`EmbeddedOnly`]): this entry point has no
    /// store handle and no way to ask for a destination. The app uses the
    /// probe/consume pair with a real sink.
    pub fn import_step_assembly(
        &mut self,
        step_text: &str,
        doc_name: &str,
        opts: StepAssemblyImport,
    ) -> Result<StepAssemblyReport, String> {
        let structured = self.probe_step_assembly(step_text)?.is_some();
        let outcome = structured.then(|| {
            let assembly = self
                .pending_step_assembly
                .take()
                .expect("a Some probe stashed the assembly it counted");
            self.consume_step_assembly(assembly, doc_name, opts.nested, &mut EmbeddedOnly)
        });
        match outcome {
            Some(Consumed::Imported(report)) => Ok(report),
            // No structure, or a structure nothing built out of: import the
            // bodies exactly as the pre-assembly lane did.
            Some(Consumed::NoComponents {
                failed_products,
                first_error,
            }) => {
                self.import_step_feature(step_text)?;
                Ok(StepAssemblyReport {
                    failed_products,
                    first_error,
                    flat_fallback: true,
                    ..StepAssemblyReport::default()
                })
            }
            None => {
                self.import_step_feature(step_text)?;
                Ok(StepAssemblyReport {
                    flat_fallback: true,
                    ..StepAssemblyReport::default()
                })
            }
        }
    }

    /// The import itself, shared by both entry points so neither has to
    /// recognise "nothing imported" from an error string.
    fn consume_step_assembly(
        &mut self,
        assembly: brep_kernel::StepAssembly,
        doc_name: &str,
        nested: bool,
        sink: &mut dyn PartSink,
    ) -> Consumed {
        let mut first_error = assembly.first_error.clone();
        // ONE writer for the whole import, so identical content is written to
        // the store exactly once however many products or LEVELS share it.
        let mut writer = PartWriter::new(sink);

        // --- what to build ------------------------------------------------
        // One row per component the USER'S document gets, each naming the
        // library entry it needs. Flat walks the whole tree to its leaves;
        // nested stops at the root's own children and folds everything below
        // each of them into that child's part document.
        let plan = if nested {
            plan_nested(&assembly, doc_name, &mut first_error, &mut writer)
        } else {
            plan_flat(&assembly, &mut first_error)
        };
        let Plan {
            wanted,
            factors,
            documents,
            mut failed_products,
            baked_below_root,
        } = plan;

        // --- build the library entries -------------------------------------
        // In (pd_ref, factor) order so an import is deterministic regardless of
        // the tree's emit order, and ONCE per key however many instances use it.
        let mut keys: Vec<PartKey> = wanted.iter().map(|(key, _, _)| *key).collect();
        keys.sort_unstable();
        keys.dedup();
        let mut entry_names: std::collections::HashMap<PartKey, String> =
            std::collections::HashMap::new();
        {
            // THE metadata bracket. `native_import_payload` seals whatever record
            // this thread's scene-metadata store holds for each name it stamps —
            // right for a snapshot of the live scene, catastrophic here: a new
            // part whose stamped face names collide with names already in THIS
            // document would silently carry the current document's metadata.
            // Scoped to the encode alone; the rebuild below stamps records the
            // document must keep, and this guard's drop would discard them.
            //
            // The nested lane's payloads are encoded inside `plan_nested`,
            // which holds a bracket of its own for exactly the same reason.
            let _isolation = brep_kernel::IsolatedSceneMetadata::begin();
            for key in &keys {
                // Nested pre-built the whole document (a sub-assembly's is a
                // recursive `{partsLibrary, features}`); flat builds the §3.2
                // native part document right here.
                let built = match documents.get(key) {
                    Some((name, document)) => install_part(name, document, &mut writer),
                    None => {
                        let product = assembly
                            .products
                            .iter()
                            .find(|product| product.pd_ref == key.0)
                            .expect("every key names a product of this assembly");
                        build_library_entry(product, factors.get(key), doc_name, &mut writer)
                    }
                };
                match built {
                    Ok(name) => {
                        entry_names.insert(*key, name);
                    }
                    Err(error) => {
                        failed_products += 1;
                        note(&mut first_error, error);
                    }
                }
            }
        }
        if entry_names.is_empty() {
            return Consumed::NoComponents {
                failed_products,
                first_error,
            };
        }

        // --- append every instance in ONE history mutation -----------------
        // `insert_component`'s rule, verbatim: ground the FIRST component only
        // when the document has none yet. Grounding a second one over-constrains
        // the next solve.
        let mut ground_next = !(0..self.history.len()).any(|index| {
            matches!(
                self.history.feature_type(index).as_deref(),
                Some("ACOMP") | Some("ASSEMBLY COMPONENT")
            )
        });
        let mut features: Vec<serde_json::Value> = Vec::with_capacity(wanted.len());
        let mut baked_nonrigid = 0usize;
        // Which component carries the geometry a STEP occurrence chain names —
        // the map the file's occurrence-scoped PMI is rewritten through, built
        // while the ids are minted because that is the only moment both halves
        // exist.
        let mut namespaces: std::collections::HashMap<Vec<usize>, String> =
            std::collections::HashMap::new();
        for (key, pose, chain) in &wanted {
            let Some(part_name) = entry_names.get(key) else {
                continue; // this product failed to encode; counted above
            };
            let transform = match brep_kernel::AffineTransform::new(*pose) {
                Ok(transform) => transform,
                Err(error) => {
                    note(&mut first_error, format!("occurrence pose: {error}"));
                    continue;
                }
            };
            if key.1 != NO_FACTOR {
                baked_nonrigid += 1;
            }
            let id = self.history.next_feature_id("ACOMP");
            namespaces
                .entry(chain.clone())
                .or_insert_with(|| format!("{id}:"));
            features.push(serde_json::json!({
                "type": "ACOMP",
                "inputParams": {
                    "id": id,
                    "partName": part_name,
                    "transform": brep_kernel::transform_to_pose_params(&transform),
                    "isFixed": ground_next,
                },
                "persistentData": {}
            }));
            ground_next = false;
        }
        if features.is_empty() {
            return Consumed::NoComponents {
                failed_products,
                first_error,
            };
        }

        // The library block must ride the request so the display runner ingests
        // the new entries on the very next run (as `insert_component` does).
        // Written only now that there are components to reference them, so an
        // import that produced nothing leaves the document untouched.
        if let Ok(library) =
            serde_json::from_str::<serde_json::Value>(&brep_kernel::parts_library_json())
        {
            self.history.set_parts_library(library);
        }
        // Frame the assembly once the (possibly async) run lands — see
        // [`EngineState::pending_fit`], same reasoning as `import_step_feature`.
        self.pending_fit = true;
        let instances = features.len();
        let baked_nonrigid = baked_nonrigid + baked_below_root;
        self.add_features(&features);

        // The file's ASSEMBLY-level PMI — annotations it pinned to an
        // occurrence, annotations spanning two parts, free notes — merged into
        // the document's own block with its references rewritten into THIS
        // import's component namespaces.
        //
        // AFTER the features, and with no checkpoint of its own, for the
        // reason `import_step_feature` reads its PMI BEFORE its `add_feature`:
        // `push_features` checkpoints at its START, so a block written first
        // is inside the snapshot undo restores — one undo would take the
        // components away and leave the annotations behind, every one of them
        // naming a component that no longer exists. Both writes have to fall
        // after the one checkpoint.
        //
        // It costs a second rebuild, which is what the flat lane already pays,
        // and only on a file that carries assembly-level PMI at all.
        //
        // A part's OWN PMI is not here: it rode into that part's document in
        // `native_part_document`, once for the part however many instances
        // place it — the exporter's rule, which is what makes the round trip
        // land the same annotations on the same parts.
        let pmi_unplaced = self.merge_assembly_pmi(&assembly, &namespaces);
        for line in &assembly.pmi_diagnostics {
            note(&mut first_error, line.clone());
        }
        if pmi_unplaced > 0 {
            note(
                &mut first_error,
                format!(
                    "{pmi_unplaced} PMI reference(s) name an occurrence this import placed no \
                     component for; the annotations are kept unresolved"
                ),
            );
        }
        Consumed::Imported(StepAssemblyReport {
            // DISTINCT entries, not distinct keys: `add_part_to_library` reuses
            // an entry whose content already matches, so two products that are
            // the same geometry collapse to one part (§3.5's free content dedup).
            parts: entry_names
                .values()
                .collect::<std::collections::HashSet<_>>()
                .len(),
            instances,
            baked_nonrigid,
            failed_products,
            first_error,
            flat_fallback: false,
        })
    }
}

impl EngineState {
    /// Merge a structured import's ASSEMBLY-level PMI into the open document's
    /// block, rewriting every occurrence-scoped reference into the component
    /// namespace this import minted for that STEP occurrence chain.
    ///
    /// Returns how many references named a chain with no component — a NESTED
    /// import folds everything below a root child into that child's own
    /// document, so an annotation the file pinned three levels down has no
    /// top-level component to name. Those references keep the chain they came
    /// with, so the row reports itself in the panel instead of silently
    /// resolving to some other instance.
    ///
    /// No rerun and no undo checkpoint of its own: the caller writes this
    /// before `add_features`, whose single rebuild resolves it.
    fn merge_assembly_pmi(
        &mut self,
        assembly: &brep_kernel::StepAssembly,
        namespaces: &std::collections::HashMap<Vec<usize>, String>,
    ) -> usize {
        let Some(lifted) = assembly.pmi.clone() else {
            return 0;
        };
        let mut unplaced = 0usize;
        let mut state = self.pmi_state();
        for view in lifted.views {
            let id = state.next_id("VIEW");
            let mut annotations = Vec::with_capacity(view.annotations.len());
            for mut annotation in view.annotations {
                unplaced += brep_kernel::rewrite_occurrence_refs(&mut annotation.params, |chain| {
                    namespaces.get(chain).cloned()
                });
                let prefix = brep_kernel::pmi_type(&annotation.kind)
                    .map(|def| def.short_name)
                    .unwrap_or("PMI");
                let fresh = state.next_id(prefix);
                if let Some(object) = annotation.params.as_object_mut() {
                    object.insert("id".into(), serde_json::Value::String(fresh));
                }
                annotations.push(annotation);
            }
            state.views.push(brep_kernel::PmiView {
                id,
                name: view.name,
                camera: view.camera,
                display: view.display,
                annotations,
            });
        }
        let block = serde_json::to_value(&state).ok();
        self.history.set_pmi_block_no_undo(block);
        // The tail has to re-resolve the merged block against the components
        // the run above created; nothing else in this lane reruns after them.
        self.rerun_history();
        unplaced
    }
}

/// What one import decided to build, before any of it is installed: the rows
/// the user's document gets, and whatever each lane needed to work out on the
/// way there.
#[derive(Default)]
struct Plan {
    /// One row per component of the USER'S document, in emit order: the
    /// library entry it needs, its pose, and the STEP occurrence chain it
    /// stands for — which is how a lifted occurrence-scoped annotation finds
    /// the component that carries its geometry.
    wanted: Vec<(PartKey, Mat4, Vec<usize>)>,
    /// FLAT only: the non-rigid factor a key's part must bake (§3.4). The
    /// nested lane bakes inside its own builder and hands the finished document
    /// over in `documents` instead.
    factors: std::collections::HashMap<PartKey, Mat4>,
    /// NESTED only: `(entry name, part document)` per key, already built — a
    /// leaf's §3.2 native document, or a sub-assembly's recursive
    /// `{partsLibrary, features}`.
    documents: std::collections::HashMap<PartKey, (String, serde_json::Value)>,
    /// Products that did not encode while planning (nested builds payloads
    /// during the plan; flat builds them during the install).
    failed_products: usize,
    /// Non-rigid occurrences baked BELOW the root — nested only, since the flat
    /// lane has no below-the-root and counts its bakes at install time.
    baked_below_root: usize,
}

/// **FLAT**: flatten the occurrence tree to its geometry-bearing nodes, each
/// carrying the COMPOSED world pose. Byte-for-byte the lane A6 shipped.
fn plan_flat(assembly: &brep_kernel::StepAssembly, first_error: &mut Option<String>) -> Plan {
    let mut plan = Plan::default();
    for placed in &compose_world_occurrences(assembly) {
        let product = &assembly.products[placed.product];
        if product.bodies.is_empty() {
            continue; // a pure assembly node contributes structure, not a component
        }
        let (key, pose) = if placed.rigid_path {
            ((product.pd_ref, NO_FACTOR), placed.world)
        } else {
            // §3.4: world = rigid · factor. Bake `factor` into a distinct part
            // and give the instance the rigid residue, so a mirrored instance
            // never lands on its unmirrored twin.
            match split_rigid(&placed.world) {
                // Non-rigid edges that cancel out along the path leave an
                // identity factor: that is an ordinary instance of the ordinary
                // part, not a bake.
                Ok((rigid, factor)) if is_identity(&factor) => {
                    ((product.pd_ref, NO_FACTOR), rigid)
                }
                Ok((rigid, factor)) => {
                    let key = (product.pd_ref, factor_key(&factor));
                    plan.factors.insert(key, factor);
                    (key, rigid)
                }
                Err(error) => {
                    note(first_error, error);
                    continue;
                }
            }
        };
        plan.wanted.push((key, pose, placed.chain.clone()));
    }
    plan
}

/// **NESTED**: the live document plays the ROOT, so it gets one component per
/// root-level row and nothing deeper —
///
/// - a root's OWN bodies become a leaf part at identity (exactly the flat
///   lane's treatment of interior geometry at the root), and
/// - each root-child occurrence becomes ONE component: a leaf part when the
///   child has no children of its own, else a rigid sub-assembly whose part
///   document carries its own `partsLibrary` and its own ACOMPs.
///
/// Emit order matches [`plan_flat`]'s DFS pre-order — root before its children,
/// children by ascending `edge_ref` — which is what makes the two lanes produce
/// the SAME document for a depth-1 tree.
fn plan_nested(
    assembly: &brep_kernel::StepAssembly,
    doc_name: &str,
    first_error: &mut Option<String>,
    writer: &mut PartWriter<'_>,
) -> Plan {
    // The same bracket the install loop holds, for the same reason: every
    // payload this builder encodes (at every level) must see an empty ambient
    // scene-metadata store, or a nested leaf whose stamped face names collide
    // with the live document's silently inherits the live document's records.
    let _isolation = brep_kernel::IsolatedSceneMetadata::begin();
    let mut build = NestedBuild {
        assembly,
        doc_name,
        writer,
        memo: std::collections::HashMap::new(),
        factors: std::collections::HashMap::new(),
        entries: 0,
        bytes: 0,
        failed_products: 0,
        baked_nonrigid: 0,
        first_error: None,
    };
    let mut plan = Plan::default();
    for &root in &assembly.roots {
        let mut rows: Vec<(DocKey, Mat4, Vec<usize>)> = Vec::new();
        // The root's OWN bodies become a leaf part at identity — exactly the
        // flat lane's treatment, and the reason a depth-1 tree comes out the
        // same either way.
        if !assembly.products[root].bodies.is_empty() {
            rows.push((
                DocKey::Leaf((assembly.products[root].pd_ref, NO_FACTOR)),
                MAT4_IDENTITY,
                Vec::new(),
            ));
        }
        // Root-level bakes are counted by the install loop's own pass over
        // `wanted` (they are ordinary top-level rows); only bakes BELOW the root
        // — which never become rows of the user's document — are counted here.
        let mut root_level_bakes = 0usize;
        build.place_children(root, &[root], &mut rows, &mut root_level_bakes);
        for (key, pose, chain) in rows {
            let part = match build.document(key, &mut vec![root]) {
                Ok(Some(document)) => document,
                // A subtree with no geometry anywhere places nothing — the flat
                // lane says the same thing by emitting no component for it.
                Ok(None) => continue,
                Err(error) => {
                    build.failed_products += 1;
                    note(&mut build.first_error, error);
                    continue;
                }
            };
            let part_key = key.part_key(assembly);
            plan.documents.insert(part_key, part);
            plan.wanted.push((part_key, pose, chain));
        }
    }
    plan.failed_products = build.failed_products;
    plan.baked_below_root = build.baked_nonrigid;
    if let Some(error) = build.first_error {
        note(first_error, error);
    }
    plan
}

/// How deep the recursive builder will go before it refuses. `read_step_assembly`
/// guards cycles inside its own walk and [`NestedBuild::document`] guards them
/// again along the recursion path, so this is the SECOND line: a malformed file
/// that is merely pathologically deep (rather than cyclic) must not run the
/// native stack out. Sixty-four levels of embedded documents is already far past
/// anything a real CAD assembly carries — and each level embeds the whole
/// subtree below it, so the document would be unusable long before then.
const MAX_NESTED_DEPTH: usize = 64;

/// How many DISTINCT part documents a nested import may build. Bounds the
/// builder's work; it does NOT bound the result's size — see
/// [`MAX_NESTED_BYTES`], which is the guard that matters.
const MAX_NESTED_ENTRIES: usize = 10_000;

/// How many bytes of part document a nested import may EMBED, summed over every
/// `partsLibrary` entry it writes at every level.
///
/// This is the guard neither the depth cap nor the entry count provides. A
/// product reachable at many different depths is stored once PER LEVEL — the
/// memo builds its document once, but each parent embeds a COPY, so a
/// diamond-shaped structure well inside the depth cap can still multiply out
/// geometrically. Charging the embedded bytes is the only place that
/// multiplication is visible, so it is charged where it happens.
const MAX_NESTED_BYTES: usize = 256 * 1024 * 1024;

/// What a nested part document is memoised under. A product is either a leaf
/// (no occurrence children) or an assembly node, never both, so the two
/// variants can never name the same product — except at a ROOT, whose own
/// bodies become a leaf part while the root itself is an assembly node. That
/// case is exactly why this is an enum and not a bare [`PartKey`].
#[derive(Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, Debug)]
enum DocKey {
    /// A geometry-bearing product placed as a part: `(pd_ref, baked factor)`.
    Leaf(PartKey),
    /// A product placed as a rigid sub-assembly, by index into `products`.
    Assembly(usize),
}

impl DocKey {
    /// The parts-library identity this document is stored under. Always keyed on
    /// the `pd_ref` (never the product INDEX, which lives in a different number
    /// space and would collide with some other product's `pd_ref`). An assembly
    /// node never carries a baked factor — a non-rigid edge into one is skipped,
    /// see [`NestedBuild::place_children`] — so `NO_FACTOR` is exact.
    fn part_key(self, assembly: &brep_kernel::StepAssembly) -> PartKey {
        match self {
            DocKey::Leaf(key) => key,
            DocKey::Assembly(product) => (assembly.products[product].pd_ref, NO_FACTOR),
        }
    }
}

/// The recursive builder behind [`plan_nested`]: turns one product into the part
/// document that represents it, bottom-up, memoised so a product reached from
/// several parents is built ONCE however many places embed it.
struct NestedBuild<'a, 'w> {
    assembly: &'a brep_kernel::StepAssembly,
    doc_name: &'a str,
    /// Where a CHILD library entry's document is written, shared with the
    /// top-level install loop so one part is one file at every level.
    writer: &'a mut PartWriter<'w>,
    /// `None` = this subtree carries no geometry at all, so nothing places it.
    memo: std::collections::HashMap<DocKey, Option<(String, serde_json::Value)>>,
    /// The non-rigid factor behind every baked [`DocKey::Leaf`] key, so the
    /// builder never has to reconstruct a matrix out of its own hash key.
    factors: std::collections::HashMap<PartKey, Mat4>,
    entries: usize,
    /// Bytes of part document embedded so far — the [`MAX_NESTED_BYTES`] charge.
    bytes: usize,
    failed_products: usize,
    baked_nonrigid: usize,
    first_error: Option<String>,
}

impl NestedBuild<'_, '_> {
    /// The part document for `key`, built once and reused. `ancestors` is the
    /// recursion path — the cycle guard, and the depth the cap is measured on.
    ///
    /// A cyclic file gets ONE deterministic truncation: the memo keeps whichever
    /// path reached a node first, and that path's skipped back-edge is the one
    /// every embedding sees. Deterministic and finite is the whole contract for
    /// input that is malformed by construction.
    fn document(
        &mut self,
        key: DocKey,
        ancestors: &mut Vec<usize>,
    ) -> Result<Option<(String, serde_json::Value)>, String> {
        if let Some(hit) = self.memo.get(&key) {
            return Ok(hit.clone());
        }
        if ancestors.len() >= MAX_NESTED_DEPTH {
            return Err(format!(
                "nested import: sub-assembly nesting deeper than {MAX_NESTED_DEPTH} levels \
                 (import as bodies, or import flat)"
            ));
        }
        let built = match key {
            DocKey::Leaf(part) => self.leaf_document(part),
            DocKey::Assembly(product) => {
                ancestors.push(product);
                let built = self.assembly_document(product, ancestors);
                ancestors.pop();
                built
            }
        }?;
        self.memo.insert(key, built.clone());
        Ok(built)
    }

    /// A geometry-bearing product as the §3.2 part document — the same one the
    /// flat lane installs, built by the same helper, so a depth-1 nested import
    /// and a flat one store byte-identical entries.
    fn leaf_document(
        &mut self,
        key: PartKey,
    ) -> Result<Option<(String, serde_json::Value)>, String> {
        let product = self
            .assembly
            .products
            .iter()
            .find(|product| product.pd_ref == key.0)
            .expect("every key names a product of this assembly");
        if product.bodies.is_empty() {
            return Ok(None);
        }
        let factor = self.factors.get(&key).copied();
        self.spend_entry()?;
        native_part_document(product, factor.as_ref(), self.doc_name).map(Some)
    }

    /// An assembly-node product as a rigid sub-assembly document: its OWN bodies
    /// as plain native IMPORT3D features (the interior-node geometry Phase 1
    /// could only make a SIBLING of its own children), one ACOMP per child
    /// occurrence, and the children's documents in this level's own
    /// `partsLibrary`.
    ///
    /// The entries carry NO snapshot. An entry with an unreadable snapshot heals
    /// from its embedded document (`assembly_component.rs`'s SELF-HEAL lane),
    /// and for a native part that heal is a decode + re-encode — so the level
    /// above bakes this whole subtree into ITS snapshot on insert, and these
    /// inner caches would only ever be rebuilt to be thrown away.
    fn assembly_document(
        &mut self,
        product: usize,
        ancestors: &mut Vec<usize>,
    ) -> Result<Option<(String, serde_json::Value)>, String> {
        let node = &self.assembly.products[product];
        let mut library = serde_json::Map::new();
        let mut features: Vec<serde_json::Value> = Vec::new();

        // The node's own bodies first, matching the flat lane's "a node before
        // its children" emit order.
        if !node.bodies.is_empty() {
            let payload = brep_kernel::native_import_payload_with_appearance(
                "IMPORT3D1",
                &node.bodies,
                &node.appearances,
            )
            .map_err(|error| format!("part '{}': {error}", part_name(node, self.doc_name)))?;
            features.push(serde_json::json!({
                "type": "IMPORT3D",
                "inputParams": { "id": "IMPORT3D1", "nativeBrep": payload },
                "persistentData": {},
            }));
        }

        // One ACOMP per child occurrence, children by ascending `edge_ref`.
        let mut rows: Vec<(DocKey, Mat4, Vec<usize>)> = Vec::new();
        let mut bakes = 0usize;
        self.place_children(product, ancestors, &mut rows, &mut bakes);
        self.baked_nonrigid += bakes;
        let mut names: std::collections::HashMap<DocKey, String> =
            std::collections::HashMap::new();
        // `add_part_to_library`'s content reuse, applied to this level's block:
        // two products that are the SAME geometry collapse to one entry (§3.5's
        // free dedup), and every instance of either references it.
        let mut by_signature: std::collections::HashMap<String, String> =
            std::collections::HashMap::new();
        let mut components = 0usize;
        // The occurrence chain is not needed at this level: a sub-assembly's
        // own ACOMPs live in ITS document, which the open document's PMI never
        // names (an annotation pinned below a root child has no top-level
        // component — `merge_assembly_pmi` reports it).
        for (key, pose, _chain) in rows {
            let name = match names.get(&key) {
                Some(name) => name.clone(),
                None => {
                    let built = match self.document(key, ancestors) {
                        Ok(Some(built)) => built,
                        Ok(None) => continue,
                        Err(error) => {
                            self.failed_products += 1;
                            note(&mut self.first_error, error);
                            continue;
                        }
                    };
                    let serialized = built.1.to_string();
                    let signature = document_signature(&serialized);
                    let name = match by_signature.get(&signature) {
                        Some(name) => name.clone(),
                        None => {
                            // Charged HERE, at the embedding, because that is
                            // where a product stored once per level multiplies.
                            self.spend_bytes(serialized.len())?;
                            // Unique WITHIN this level's library — parent and
                            // child libraries are independent, so a name
                            // taken upstairs is free down here.
                            let name = unique_entry_name(&library, &built.0);
                            // A nested child is a part like any other: it gets
                            // its own store document and a REAL sourceKey, so
                            // Open Part and update-components work the same way
                            // however deep it sits.
                            let source_key =
                                self.writer.key_for(&name, &serialized, &signature);
                            library.insert(
                                name.clone(),
                                serde_json::json!({
                                    "sourceKey": source_key,
                                    "sourceSignature": signature.clone(),
                                    "document": built.1,
                                    "snapshot": "",
                                }),
                            );
                            by_signature.insert(signature, name.clone());
                            name
                        }
                    };
                    names.insert(key, name.clone());
                    name
                }
            };
            let Ok(transform) = brep_kernel::AffineTransform::new(pose) else {
                note(
                    &mut self.first_error,
                    format!("sub-assembly '{name}': occurrence pose is not an affine"),
                );
                continue;
            };
            components += 1;
            features.push(serde_json::json!({
                "type": "ACOMP",
                "inputParams": {
                    // Its OWN counter, so the ids read `ACOMP1..n` whether or
                    // not this node also owns bodies. (The id must match
                    // `ACOMP<digits>`: it IS the namespace prefix.)
                    "id": format!("ACOMP{components}"),
                    "partName": name,
                    "transform": brep_kernel::transform_to_pose_params(&transform),
                    // Written EXPLICITLY rather than left to the kernel's
                    // auto-ground rule, which keys on ABSENCE: the first
                    // component of an assembly is grounded, and every other one
                    // must not be, or the next solve is over-constrained.
                    "isFixed": components == 1,
                },
                "persistentData": {},
            }));
        }

        // A node whose whole subtree failed to produce geometry places nothing.
        // Returning `None` rather than a feature-less document matters: an empty
        // document is a hard error inside `add_part_to_library`, which would turn
        // "there was nothing here" into "the import failed".
        if features.is_empty() {
            return Ok(None);
        }
        self.spend_entry()?;
        let mut document =
            serde_json::json!({ "partsLibrary": library, "features": features });
        // A sub-assembly is a part too: its own `PRODUCT.id` is the part number
        // the level above exports it under, and its own PMI — the annotations
        // on the bodies IT owns, not its children's — is its document's.
        if let Some(attributes) = part_attributes(node) {
            document[PART_ATTRIBUTES] = attributes;
        }
        if let Some(pmi) = &node.pmi {
            if let Ok(block) = serde_json::to_value(pmi) {
                document["pmi"] = block;
            }
        }
        Ok(Some((part_name(node, self.doc_name), document)))
    }

    /// The child occurrences of `product`, as `(document key, pose, occurrence
    /// chain)` rows in the kernel walk's order — ascending `edge_ref`, with the
    /// same ancestor cycle guard. The pose is the occurrence's own child→parent
    /// placement: nesting is precisely what stops it having to be composed.
    ///
    /// The chain is one edge long here by construction: a nested import's rows
    /// are the ROOT's own children, and everything below one of them is folded
    /// into that child's document.
    fn place_children(
        &mut self,
        product: usize,
        ancestors: &[usize],
        rows: &mut Vec<(DocKey, Mat4, Vec<usize>)>,
        bakes: &mut usize,
    ) {
        let mut children: Vec<&brep_kernel::StepOccurrence> = self
            .assembly
            .occurrences
            .iter()
            .filter(|occurrence| occurrence.parent == product)
            .collect();
        children.sort_by_key(|occurrence| occurrence.edge_ref);
        for occurrence in children {
            if ancestors.contains(&occurrence.child) {
                note(
                    &mut self.first_error,
                    format!(
                        "occurrence #{} closes a cycle in the product structure and was skipped",
                        occurrence.edge_ref
                    ),
                );
                continue;
            }
            let child = &self.assembly.products[occurrence.child];
            let is_assembly = self
                .assembly
                .occurrences
                .iter()
                .any(|edge| edge.parent == occurrence.child);
            if occurrence.rigid {
                let key = if is_assembly {
                    DocKey::Assembly(occurrence.child)
                } else {
                    DocKey::Leaf((child.pd_ref, NO_FACTOR))
                };
                rows.push((key, occurrence.placement, vec![occurrence.edge_ref]));
                continue;
            }
            // §3.4 on a single edge: a leaf bakes its non-rigid factor into its
            // own part, exactly as the flat lane does with the composed pose.
            match split_rigid(&occurrence.placement) {
                Ok((rigid, factor)) if is_identity(&factor) => {
                    let key = if is_assembly {
                        DocKey::Assembly(occurrence.child)
                    } else {
                        DocKey::Leaf((child.pd_ref, NO_FACTOR))
                    };
                    rows.push((key, rigid, vec![occurrence.edge_ref]));
                }
                // A mirrored/scaled SUB-ASSEMBLY would have to push its factor
                // down through a whole document tree, rewriting every level's
                // poses. Nothing in the corpus does it, and a wrong answer here
                // would be a silently mis-handed assembly: skip and say so, so
                // the user can re-import flat (which bakes it correctly).
                Ok(_) if is_assembly => {
                    note(
                        &mut self.first_error,
                        format!(
                            "occurrence #{} places sub-assembly '{}' with a non-rigid transform, \
                             which a nested import cannot represent — import flat instead",
                            occurrence.edge_ref,
                            part_name(child, self.doc_name)
                        ),
                    );
                }
                Ok((rigid, factor)) => {
                    *bakes += 1;
                    let key = (child.pd_ref, factor_key(&factor));
                    self.factors.insert(key, factor);
                    rows.push((DocKey::Leaf(key), rigid, vec![occurrence.edge_ref]));
                }
                Err(error) => note(&mut self.first_error, error),
            }
        }
    }

    /// Charge one built part document against [`MAX_NESTED_ENTRIES`].
    fn spend_entry(&mut self) -> Result<(), String> {
        self.entries += 1;
        if self.entries > MAX_NESTED_ENTRIES {
            return Err(format!(
                "nested import: more than {MAX_NESTED_ENTRIES} distinct parts \
                 (import as bodies, or import flat)"
            ));
        }
        Ok(())
    }

    /// Charge one embedded part document against [`MAX_NESTED_BYTES`].
    fn spend_bytes(&mut self, bytes: usize) -> Result<(), String> {
        self.bytes = self.bytes.saturating_add(bytes);
        if self.bytes > MAX_NESTED_BYTES {
            return Err(format!(
                "nested import: the embedded sub-assembly documents exceed \
                 {} MB (import as bodies, or import flat)",
                MAX_NESTED_BYTES / (1024 * 1024)
            ));
        }
        Ok(())
    }
}

/// A part name not yet used in THIS level's library: `requested`, else
/// `requested-2`, `requested-3`, … — the kernel `parts_library::unique_name`
/// convention, applied to an embedded block the kernel never sees inserted.
fn unique_entry_name(library: &serde_json::Map<String, serde_json::Value>, requested: &str) -> String {
    if !library.contains_key(requested) {
        return requested.to_string();
    }
    (2..)
        .map(|counter| format!("{requested}-{counter}"))
        .find(|candidate| !library.contains_key(candidate))
        .expect("the counter loop is unbounded")
}

/// Keep the FIRST thing that went wrong (the report carries one, and the first
/// is the one that explains the rest).
fn note(slot: &mut Option<String>, error: String) {
    if slot.is_none() {
        *slot = Some(error);
    }
}

/// Encode one product as a parts-library entry and return the EFFECTIVE entry
/// name the instances must reference (`add_part_to_library` disambiguates a name
/// clash and REUSES an entry with identical content, which is where cross-import
/// dedup comes from).
///
/// `factor`, when present, is the non-rigid part of an occurrence's placement:
/// applied to the geometry HERE, so the instance can carry a rigid pose (§3.4).
fn build_library_entry(
    product: &brep_kernel::StepProduct,
    factor: Option<&Mat4>,
    doc_name: &str,
    writer: &mut PartWriter<'_>,
) -> Result<String, String> {
    let (name, document) = native_part_document(product, factor, doc_name)?;
    install_part(&name, &document, writer)
}

/// The §3.2 part document for one product's OWN bodies: ONE IMPORT3D whose only
/// input is the native payload, plus the library name it wants. No STEP text is
/// stored anywhere — a rebuild of this part is a base64 decode, not a re-parse.
///
/// `factor`, when present, is the non-rigid part of an occurrence's placement:
/// applied to the geometry HERE, so the instance can carry a rigid pose (§3.4).
///
/// Split out from [`build_library_entry`] because the nested lane needs the
/// DOCUMENT before it installs anything — a leaf's document is embedded in its
/// parent's `partsLibrary`, where there is no `add_part_to_library` to call.
/// One producer, so a leaf part is byte-identical however deep it lands.
fn native_part_document(
    product: &brep_kernel::StepProduct,
    factor: Option<&Mat4>,
    doc_name: &str,
) -> Result<(String, serde_json::Value), String> {
    let mut name = part_name(product, doc_name);
    let bodies = match factor {
        None => product.bodies.clone(),
        Some(factor) => {
            let transform = brep_kernel::AffineTransform::new(*factor)
                .map_err(|error| format!("part '{name}': non-rigid factor: {error}"))?;
            let mirrored = transform.determinant3() < 0.0;
            name.push_str(if mirrored { " (mirrored)" } else { " (scaled)" });
            product
                .bodies
                .iter()
                .map(|body| {
                    // A mirror MUST reverse orientation or `transform_brep`
                    // refuses it (an unreversed reflection inverts the solid).
                    brep_kernel::transform_brep(body, transform, mirrored)
                        .map_err(|error| format!("part '{name}': {error}"))
                })
                .collect::<Result<Vec<_>, _>>()?
        }
    };
    // The product's STEP colours ride into the payload with the geometry (the
    // snapshot captures the records the stamp writes), so a coloured part keeps
    // its colour through the parts library and every reload.
    let payload = brep_kernel::native_import_payload_with_appearance(
        "IMPORT3D1",
        &bodies,
        &product.appearances,
    )
    .map_err(|error| format!("part '{name}': {error}"))?;
    let mut document = serde_json::json!({
        "features": [{
            "type": "IMPORT3D",
            "inputParams": { "id": "IMPORT3D1", "nativeBrep": payload },
            "persistentData": {},
        }]
    });
    if let Some(attributes) = part_attributes(product) {
        document[PART_ATTRIBUTES] = attributes;
    }
    // The part's own PMI, lifted once for the PART (`assembly_pmi.rs`). Its
    // references already name what the `IMPORT3D1` feature above stamps, and a
    // baked factor transforms the bodies without reordering their faces, so a
    // mirrored variant's names hold too.
    if let Some(pmi) = &product.pmi {
        if let Ok(block) = serde_json::to_value(pmi) {
            document["pmi"] = block;
        }
    }
    Ok((name, document))
}

/// The BOM attribute record an imported product's document carries: its STEP
/// `PRODUCT.id` as `Part_Number`, or `None` when the file gave it none.
///
/// This is the OTHER half of `io/step/export_tree.rs`, which writes a part's
/// `partAttributes.Part_Number` back out as `PRODUCT.id` — the identity a
/// downstream PDM keys on. Without it an imported id died at the import: the
/// document had no part number, so a re-export fell back to the library entry's
/// `sourceKey` and the vendor's own part numbers were replaced by our file
/// names on the first round trip.
///
/// It is only ever set on a document being MINTED here (a fresh part document
/// for one product), so it cannot overwrite a part number a user typed: a part
/// document that already exists is reused by
/// [`add_part_to_library`](brep_kernel::add_part_to_library)'s content match and
/// never rebuilt through this path.
///
/// Trimmed and blank-filtered to match the reader's own `part_number`, so a
/// `PRODUCT('  ','bolt',…)` does not mint a whitespace part number that would
/// then export as one.
fn part_attributes(product: &brep_kernel::StepProduct) -> Option<serde_json::Value> {
    let number = product.id.trim();
    if number.is_empty() {
        return None;
    }
    Some(serde_json::json!({ brep_kernel::PART_NUMBER: number }))
}

/// Install a part document as a parts-library entry of the OPEN document and
/// return the EFFECTIVE entry name the instances must reference
/// (`add_part_to_library` disambiguates a name clash and REUSES an entry with
/// identical content, which is where cross-import dedup comes from).
///
/// The `sourceKey` comes from the [`PartSink`]: an imported part is written to
/// the store as its own document and carries a REAL key, exactly like a part
/// inserted from the parts library, so there is no second kind of part. A sink
/// that declines (no store, or a failed write) yields `""` — the embedded-only
/// entry this lane used to produce unconditionally, and the case
/// `UpdateComponents` already skips.
fn install_part(
    name: &str,
    document: &serde_json::Value,
    writer: &mut PartWriter<'_>,
) -> Result<String, String> {
    let document = document.to_string();
    let signature = document_signature(&document);
    let source_key = writer.key_for(name, &document, &signature);
    brep_kernel::add_part_to_library_impl(name, &source_key, &signature, &document)
        .map_err(|error| format!("part '{name}': {error}"))
}

/// Where an imported assembly's unique parts are written, so each becomes a
/// document in its own right rather than a payload embedded in one assembly.
///
/// A trait, and not a `&dyn ModelStore`, because the store lives in `BREP_app`
/// and this crate is BELOW it — `BREP_app` depends on `BREP_render`, so naming
/// the store here would be a dependency cycle. The import therefore asks for a
/// key and the app answers with one, which is also what keeps the destination
/// (and any prompt for it) entirely the app's business.
pub trait PartSink {
    /// Store `document_json` under a name derived from `part_name` and return
    /// the stable key it can be read back by. `None` declines — no store, or a
    /// write that failed — and the entry stays embedded-only.
    fn store_part(&mut self, part_name: &str, document_json: &str) -> Option<String>;
}

/// The sink that stores nothing: every entry stays embedded-only. The default
/// for headless callers and tests, which have no store to write to.
pub struct EmbeddedOnly;

impl PartSink for EmbeddedOnly {
    fn store_part(&mut self, _part_name: &str, _document_json: &str) -> Option<String> {
        None
    }
}

/// A [`PartSink`] plus the CONTENT DEDUP that must ride with it.
///
/// `add_part_to_library` reuses an entry whose `(sourceKey, sourceSignature)`
/// both match, which is where §3.5's free dedup came from while every imported
/// part carried the same empty key. Give each part its own key and that reuse
/// stops: the same product under two `PRODUCT_DEFINITION`s would become two
/// entries AND two identical files.
///
/// So the dedup moves in front of the write, keyed on the document signature
/// alone. Identical content is written ONCE and every occurrence of it gets the
/// SAME key — which then makes `add_part_to_library`'s own `(key, signature)`
/// reuse fire exactly as before. Dedup ACROSS imports keeps working for the
/// same reason: a re-import derives the same file name, so the same key and
/// signature come back and the resident entry is reused.
struct PartWriter<'a> {
    sink: &'a mut dyn PartSink,
    by_signature: std::collections::HashMap<String, String>,
}

impl<'a> PartWriter<'a> {
    fn new(sink: &'a mut dyn PartSink) -> Self {
        Self {
            sink,
            by_signature: std::collections::HashMap::new(),
        }
    }

    /// The `sourceKey` for a part with this content — writing it exactly once
    /// however many products share it.
    fn key_for(&mut self, name: &str, document_json: &str, signature: &str) -> String {
        if let Some(key) = self.by_signature.get(signature) {
            return key.clone();
        }
        let key = self
            .sink
            .store_part(name, document_json)
            .unwrap_or_default();
        self.by_signature.insert(signature.to_string(), key.clone());
        key
    }
}

/// An sRGB triple in 0..=1 as the 8-bit channels `#RRGGBB` holds — the form
/// [`brep_kernel::StepColors`] takes, and the same `round(c * 255)` conversion
/// `brep_kernel::ImportedColor::to_hex` uses, so a colour written to a STEP file
/// is the colour the metadata record spells.
fn srgb_bytes(rgb: [f64; 3]) -> [u8; 3] {
    rgb.map(|channel| (channel.clamp(0.0, 1.0) * 255.0).round() as u8)
}

/// The library name for a product: its `PRODUCT.name`, else a stem built from
/// the imported document's name so an unnamed product is still identifiable.
fn part_name(product: &brep_kernel::StepProduct, doc_name: &str) -> String {
    let named = product.name.trim();
    if !named.is_empty() {
        return named.to_string();
    }
    match doc_name.trim() {
        "" => format!("part-{}", product.pd_ref),
        stem => format!("{stem}-part-{}", product.pd_ref),
    }
}

/// The dialog's counts, taken from the SAME walk the import runs, so the numbers
/// the user was shown are the numbers they get (bar an encode failure, and bar
/// the extra entry a non-rigid occurrence bakes).
pub(super) fn probe_counts(assembly: &brep_kernel::StepAssembly) -> StepAssemblyProbe {
    let mut parts = std::collections::HashSet::new();
    let mut instances = 0usize;
    let mut nested_depth = 0usize;
    for placed in compose_world_occurrences(assembly) {
        let product = &assembly.products[placed.product];
        if product.bodies.is_empty() {
            continue;
        }
        parts.insert(product.pd_ref);
        instances += 1;
        nested_depth = nested_depth.max(placed.depth);
    }
    StepAssemblyProbe {
        parts: parts.len(),
        instances,
        nested_depth,
    }
}

/// Depth-first from the roots, composing each occurrence's child→parent
/// placement into a world transform — the consumer half of `read_step_assembly`,
/// which deliberately transforms nothing.
///
/// Emit order, child ordering (by `edge_ref`) and the ancestor cycle guard mirror
/// the kernel's own `walk_occurrences`, which is what makes the components this
/// lane produces the same solids, in the same order, as the flat lane's — the
/// kernel asserts that equivalence BIT-for-bit
/// (`step_import/tests/assembly_structure.rs`), and
/// `structured_import_matches_the_flat_lane_geometry` below re-asserts it from
/// this side, where a divergence would actually land.
fn compose_world_occurrences(assembly: &brep_kernel::StepAssembly) -> Vec<PlacedProduct> {
    struct Node {
        placed: PlacedProduct,
        ancestors: Vec<usize>,
    }
    let mut out = Vec::new();
    let mut stack: Vec<Node> = assembly
        .roots
        .iter()
        .rev()
        .map(|&product| Node {
            placed: PlacedProduct {
                product,
                world: MAT4_IDENTITY,
                depth: 0,
                rigid_path: true,
                chain: Vec::new(),
            },
            ancestors: vec![product],
        })
        .collect();
    while let Some(node) = stack.pop() {
        let (product, world, depth, rigid_path, chain) = (
            node.placed.product,
            node.placed.world,
            node.placed.depth,
            node.placed.rigid_path,
            node.placed.chain.clone(),
        );
        out.push(node.placed);
        let mut children: Vec<&brep_kernel::StepOccurrence> = assembly
            .occurrences
            .iter()
            .filter(|occurrence| occurrence.parent == product)
            .collect();
        children.sort_by_key(|occurrence| occurrence.edge_ref);
        for occurrence in children.into_iter().rev() {
            if node.ancestors.contains(&occurrence.child) {
                continue; // the cycle guard the kernel's walk applies
            }
            let mut ancestors = node.ancestors.clone();
            ancestors.push(occurrence.child);
            let mut child_chain = chain.clone();
            child_chain.push(occurrence.edge_ref);
            stack.push(Node {
                placed: PlacedProduct {
                    product: occurrence.child,
                    world: mat4_mul(&world, &occurrence.placement),
                    depth: depth + 1,
                    // The kernel's per-edge rigidity flag, carried down the path:
                    // a composed pose is a component pose only when every edge
                    // on the way to it was one.
                    rigid_path: rigid_path && occurrence.rigid,
                    chain: child_chain,
                },
                ancestors,
            });
        }
    }
    out
}

const MAT4_IDENTITY: Mat4 = [
    1.0, 0.0, 0.0, 0.0, //
    0.0, 1.0, 0.0, 0.0, //
    0.0, 0.0, 1.0, 0.0, //
    0.0, 0.0, 0.0, 1.0,
];

/// Row-major 4×4 product `a · b`.
fn mat4_mul(a: &Mat4, b: &Mat4) -> Mat4 {
    let mut out = [0.0; 16];
    for row in 0..4 {
        for column in 0..4 {
            out[row * 4 + column] = (0..4)
                .map(|k| a[row * 4 + k] * b[k * 4 + column])
                .sum();
        }
    }
    out
}

/// Split a non-rigid world placement into `world = rigid · factor`, where
/// `rigid` is a component pose (rotation + translation, det +1) and `factor` is
/// a purely linear residue carrying the mirror/scale/shear.
///
/// Gram-Schmidt on the linear block's columns gives `A = Q·U` with `U` upper
/// triangular and positively-diagonalled; when `Q` came out left-handed the pair
/// is re-signed through `D = diag(-1, 1, 1)` (`Q' = Q·D`, `U' = D·U`, still
/// `Q'U' = A`) so the ROTATION is a rotation and the reflection rides in the
/// factor. A mirror composed with a rotation therefore yields the same factor
/// whatever the rotation, which keeps every such instance on ONE baked part.
fn split_rigid(world: &Mat4) -> Result<(Mat4, Mat4), String> {
    let column = |index: usize| [world[index], world[4 + index], world[8 + index]];
    let dot = |a: [f64; 3], b: [f64; 3]| a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    let axpy = |a: [f64; 3], scale: f64, b: [f64; 3]| {
        [a[0] - scale * b[0], a[1] - scale * b[1], a[2] - scale * b[2]]
    };
    let (a1, a2, a3) = (column(0), column(1), column(2));

    let r11 = dot(a1, a1).sqrt();
    let mut q1 = normalize(a1, r11)?;
    let r12 = dot(q1, a2);
    let v2 = axpy(a2, r12, q1);
    let r22 = dot(v2, v2).sqrt();
    let q2 = normalize(v2, r22)?;
    let r13 = dot(q1, a3);
    let r23 = dot(q2, a3);
    let v3 = axpy(axpy(a3, r13, q1), r23, q2);
    let r33 = dot(v3, v3).sqrt();
    let q3 = normalize(v3, r33)?;

    // det Q = q1 · (q2 × q3); -1 means Q is a reflection, not a rotation.
    let cross = [
        q2[1] * q3[2] - q2[2] * q3[1],
        q2[2] * q3[0] - q2[0] * q3[2],
        q2[0] * q3[1] - q2[1] * q3[0],
    ];
    let (mut r11, mut r12, mut r13) = (r11, r12, r13);
    if dot(q1, cross) < 0.0 {
        q1 = [-q1[0], -q1[1], -q1[2]];
        r11 = -r11;
        r12 = -r12;
        r13 = -r13;
    }
    let rigid = [
        q1[0], q2[0], q3[0], world[3], //
        q1[1], q2[1], q3[1], world[7], //
        q1[2], q2[2], q3[2], world[11], //
        0.0, 0.0, 0.0, 1.0,
    ];
    let factor = [
        r11, r12, r13, 0.0, //
        0.0, r22, r23, 0.0, //
        0.0, 0.0, r33, 0.0, //
        0.0, 0.0, 0.0, 1.0,
    ];
    Ok((rigid, factor))
}

/// Unit vector, or a clear error for the degenerate column a near-singular
/// placement produces (skipped and counted, never fatal).
fn normalize(vector: [f64; 3], length: f64) -> Result<[f64; 3], String> {
    if !(length > 1e-12) || !length.is_finite() {
        return Err("occurrence placement is singular (a degenerate axis)".into());
    }
    Ok([vector[0] / length, vector[1] / length, vector[2] / length])
}

/// Is this affine the identity to 1e-9 — the tolerance the kernel's own
/// rigidity gate uses?
fn is_identity(matrix: &Mat4) -> bool {
    matrix
        .iter()
        .zip(MAT4_IDENTITY.iter())
        .all(|(value, want)| (value - want).abs() <= 1e-9)
}

/// The linear block of a baked factor as an exact bit key — two occurrences
/// share a baked part only when their factor is bit-identical, so a wrong-handed
/// reuse is not reachable through rounding.
fn factor_key(factor: &Mat4) -> [u64; 9] {
    let mut key = [0u64; 9];
    for (slot, index) in key.iter_mut().zip([0, 1, 2, 4, 5, 6, 8, 9, 10]) {
        *slot = factor[index].to_bits();
    }
    key
}

/// The product, and the placement of it, that holds a board document's board
/// in a STRUCTURED export. The board's bodies keep their own names inside it
/// (`PCB_Board`, `PCB_F.Cu/GND#3`). A component's reference designator is
/// never taken for an occurrence label when it is this word.
const BOARD_OCCURRENCE: &str = "PCB";

/// A `PRODUCT.id` a receiving system can use. A library part with no BOM part
/// number is identified by the `sourceKey` it was imported from, which is an
/// ABSOLUTE path on the exporting machine (`/tmp/…/kicad/symbols/lm358.nbrep`):
/// meaningless anywhere else, and a leak of the user's directory layout into
/// every file they send. Such an id becomes the file's stem (`lm358`), or the
/// product's name when the path has no usable stem. An id with no path
/// separator — a real part number — is kept as it is.
fn portable_product_id(id: &str, name: &str) -> String {
    const SEPARATORS: [char; 2] = ['/', '\\'];
    if !id.contains(SEPARATORS) {
        return id.to_string();
    }
    let file = id.rsplit(SEPARATORS).next().unwrap_or_default();
    let stem = file.rsplit_once('.').map_or(file, |(stem, _)| stem);
    if stem.is_empty() {
        name.to_string()
    } else {
        stem.to_string()
    }
}

