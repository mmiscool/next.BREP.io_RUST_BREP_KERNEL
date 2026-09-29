//! An isolated, disposable reconstruction session for an imported MESH file —
//! STL or 3MF, which reach the same reconstruction chain. Accept transfers the
//! exact previewed STEP into the destination document as one ordinary import
//! feature.
use crate::automation::hit_keys::HitKeyDoc;
use brep_render::engine_state::EngineState;
use brep_render::runner::{
    ConversionPolicy, MeshImportFormat, StlConversionOptions, StlConversionOutput,
};
use eframe::egui;
use std::collections::BTreeMap;

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum PreviewAction {
    None,
    Accept,
    Cancel,
}

pub struct StlImportPreview {
    pub destination: u64,
    name: String,
    /// Which reader the runner parses `bytes` with. The tolerances, the
    /// preview and the accept path are identical for every mesh format.
    format: MeshImportFormat,
    bytes: Vec<u8>,
    pub engine: EngineState,
    options: StlConversionOptions,
    revision: u64,
    pending: Option<(u64, u64)>,
    result: Option<(u64, StlConversionOutput)>,
    error: Option<String>,
    scene_changed: bool,
    hits: BTreeMap<String, [f32; 4]>,
}

impl StlImportPreview {
    pub fn new(
        destination: u64,
        format: MeshImportFormat,
        name: String,
        bytes: Vec<u8>,
        mut engine: EngineState,
    ) -> Self {
        engine.settings.wireframe = false;
        let mut preview = Self {
            destination,
            name,
            format,
            bytes,
            engine,
            options: Default::default(),
            revision: 0,
            pending: None,
            result: None,
            error: None,
            scene_changed: true,
            hits: BTreeMap::new(),
        };
        preview.rebuild();
        preview
    }

    fn busy(&self) -> bool {
        self.pending.is_some() || self.engine.run_pending()
    }

    pub fn ready(&self) -> bool {
        !self.busy()
            && self.error.is_none()
            && self
                .result
                .as_ref()
                .is_some_and(|(revision, _)| *revision == self.revision)
            && !self.engine.scene.solids().is_empty()
    }

    pub fn step_text(&self) -> Option<&str> {
        self.ready()
            .then(|| self.result.as_ref().unwrap().1.step_text.as_str())
    }

    pub fn accept_into(
        &self,
        destination_id: u64,
        destination: &mut EngineState,
    ) -> Result<(), String> {
        if destination_id != self.destination {
            return Err(
                "The destination document changed; cancel this preview and import again.".into(),
            );
        }
        let step = self
            .step_text()
            .ok_or("Update the preview before accepting the import.")?;
        destination.import_step_feature(step).map(|_| ())
    }

    fn rebuild(&mut self) {
        if self.busy() {
            return;
        }
        self.error = None;
        match self.engine.reconstruct_mesh_preview(
            self.format,
            self.bytes.clone(),
            self.options.clone(),
        ) {
            Ok(id) => self.pending = Some((id, self.revision)),
            Err(error) => self.error = Some(error),
        }
    }

    fn pump(&mut self) {
        self.engine.pump();
        while let Some(reply) = self.engine.take_mesh_preview() {
            let Some((id, revision)) = self.pending else {
                continue;
            };
            if reply.id != id {
                continue;
            }
            self.pending = None;
            match reply.result {
                Ok(output) => {
                    let document = serde_json::json!({
                        "features": [{"type": "IMPORT3D", "inputParams": {
                            "id": "IMPORT3D1", "stepText": output.step_text
                        }}], "featureCounter": 1
                    });
                    match self.engine.load_model_and_fit(&document.to_string()) {
                        Ok(_) => {
                            self.result = Some((revision, output));
                            self.scene_changed = true;
                        }
                        Err(error) => self.error = Some(error),
                    }
                }
                Err(error) => self.error = Some(error),
            }
        }
        if !self.busy()
            && self.result.is_some()
            && self.engine.scene.solids().is_empty()
            && self.error.is_none()
        {
            self.error = Some("The reconstructed BREP could not be displayed. Adjust the settings and update the preview.".into());
        }
    }

    fn hit(&mut self, key: &str, response: &egui::Response) {
        let r = response.rect;
        self.hits
            .insert(key.into(), [r.min.x, r.min.y, r.width(), r.height()]);
    }

    pub fn show(
        &mut self,
        ui: &mut egui::Ui,
        viewport: &mut crate::viewport::Viewport,
    ) -> PreviewAction {
        self.pump();
        self.hits.clear();
        let mut action = PreviewAction::None;
        egui::containers::panel::Panel::left("stl-preview-settings")
            .resizable(true)
            .default_size(350.0)
            .size_range(300.0..=550.0)
            .show(ui, |ui| {
                egui::ScrollArea::vertical().show(ui, |ui| {
                    action = self.controls(ui);
                });
            });
        if self.scene_changed {
            viewport.forget_document();
            self.scene_changed = false;
        }
        viewport.show(ui, &mut self.engine);
        if self.busy() {
            ui.ctx().request_repaint();
        }
        if ui
            .ctx()
            .input_mut(|input| input.consume_key(egui::Modifiers::NONE, egui::Key::Escape))
        {
            action = PreviewAction::Cancel;
        }
        action
    }

    fn controls(&mut self, ui: &mut egui::Ui) -> PreviewAction {
        ui.heading(format!("{} import preview", format_label(self.format)));
        ui.label(&self.name);
        ui.weak("Orbit and zoom to inspect the reconstructed BREP. Distances are in mm.");
        ui.separator();
        let mut changed = false;
        egui::Grid::new("stl-tolerances").num_columns(2).spacing([8.0, 8.0]).show(ui, |ui| {
            changed |= number(ui, &mut self.hits, "distance", "Surface distance", &mut self.options.recognition.distance_tolerance, 1e-12..=1e6, 1e-6,
                "Maximum absolute distance from mesh samples to a recognized surface. Binary STL coordinate precision sets a minimum.");
            changed |= number(ui, &mut self.hits, "relative", "Relative distance", &mut self.options.recognition.relative_tolerance, 0.0..=1.0, 1e-8,
                "Additional recognition tolerance relative to the model size.");
            let mut normal = self.options.recognition.normal_tolerance.to_degrees();
            if number(ui, &mut self.hits, "normal", "Normal angle (°)", &mut normal, 0.01..=89.0, 0.1,
                "Maximum angle between a recognized surface normal and the mesh normal.") {
                self.options.recognition.normal_tolerance = normal.to_radians(); changed = true;
            }
            let mut feature = self.options.recognition.feature_angle.to_degrees();
            if number(ui, &mut self.hits, "feature", "Feature angle (°)", &mut feature, 0.1..=89.0, 0.1,
                "Edges sharper than this angle split surface regions during recognition and BREP reconstruction.") {
                self.options.recognition.feature_angle = feature.to_radians();
                self.options.kernel_deflection_angle_degrees = feature; changed = true;
            }
            changed |= number(ui, &mut self.hits, "fit", "BREP relative fit", &mut self.options.kernel_fit_tolerance, 1e-12..=0.1, 1e-5,
                "Surface fitting tolerance for BREP reconstruction, relative to the bounding-box diagonal. Shared boundaries must also pass stricter geometry checks.");
            changed |= number(ui, &mut self.hits, "brepNormal", "BREP normal angle (°)", &mut self.options.kernel_normal_tolerance_degrees, 0.01..=89.0, 0.1,
                "Normal agreement required when grouping mesh triangles into BREP faces.");
        });
        ui.add_space(8.0);
        let mut auto_weld = self.options.weld_tolerance < 0.0;
        let auto = ui.checkbox(&mut auto_weld, "Automatic vertex weld tolerance");
        self.hit("autoWeld", &auto);
        if auto.changed() {
            self.options.weld_tolerance = if auto_weld { -1.0 } else { 1e-6 };
            changed = true;
        }
        if !auto_weld {
            egui::Grid::new("stl-weld").num_columns(2).show(ui, |ui| {
                changed |= number(ui, &mut self.hits, "weld", "Vertex weld distance", &mut self.options.weld_tolerance, 0.0..=1e3, 1e-6,
                    "Merge vertices closer than this distance. Zero merges identical coordinates only.");
            });
        }
        let mut strict = self.options.policy == ConversionPolicy::RequireFullyAnalytic;
        let response = ui
            .checkbox(&mut strict, "Require fully analytic surfaces")
            .on_hover_text("Reject reconstruction if any region must remain faceted.");
        self.hit("strict", &response);
        if response.changed() {
            self.options.policy = if strict {
                ConversionPolicy::RequireFullyAnalytic
            } else {
                ConversionPolicy::AllowFacetedFallback
            };
            changed = true;
        }
        let reset = ui.button("Reset tolerances");
        self.hit("reset", &reset);
        if reset.clicked() {
            self.options = Default::default();
            changed = true;
        }
        if changed {
            self.revision += 1;
        }
        ui.separator();
        let update = ui.add_enabled(!self.busy(), egui::Button::new("Update preview"));
        self.hit("update", &update);
        if update.clicked() {
            self.rebuild();
        }
        // Accept and Cancel sit HERE, above everything a reconstruction adds
        // or removes (the spinner, the warnings, the result's report): a row
        // drawn below them jumped down the moment a result landed, so a click
        // on Cancel in that frame hit whatever had moved into its place — and
        // the dialog stayed open (the stl-preview script, under load).
        let mut action = PreviewAction::None;
        ui.horizontal(|ui| {
            let accept = ui.add_enabled(self.ready(), egui::Button::new("Accept import"));
            self.hit("accept", &accept);
            if accept.clicked() {
                action = PreviewAction::Accept;
            }
            let cancel = ui.button("Cancel");
            self.hit("cancel", &cancel);
            if cancel.clicked() {
                action = PreviewAction::Cancel;
            }
        });
        if self.busy() {
            ui.horizontal(|ui| {
                ui.spinner();
                ui.label("Reconstructing BREP…");
            });
        }
        if self
            .result
            .as_ref()
            .is_some_and(|(revision, _)| *revision != self.revision)
        {
            ui.colored_label(
                ui.visuals().warn_fg_color,
                "Settings changed — update the preview before accepting.",
            );
        }
        if let Some(error) = &self.error {
            ui.colored_label(ui.visuals().error_fg_color, error);
        }
        if let Some((_, output)) = &self.result {
            let report = &output.report;
            ui.add_space(8.0);
            ui.label(format!(
                "{} BREP faces · {} solid(s)",
                report.exported_advanced_faces, report.roundtrip_solids
            ));
            let cylinders = output.step_text.matches("CYLINDRICAL_SURFACE(").count();
            let cones = output.step_text.matches("CONICAL_SURFACE(").count();
            let spheres = output.step_text.matches("SPHERICAL_SURFACE(").count();
            ui.label(format!(
                "{cylinders} cylindrical · {cones} conical · {spheres} spherical surfaces"
            ));
            let facets = report
                .hybrid_rebuild
                .map(|r| r.faceted_faces)
                .or(report.faceted_faces_after_merge)
                .unwrap_or(0);
            if facets > 0 {
                ui.colored_label(
                    ui.visuals().warn_fg_color,
                    format!("{facets} faces remain faceted"),
                );
            }
            ui.weak(format!(
                "Effective surface distance: {:.3e} mm",
                report.effective_distance_tolerance
            ));
            ui.collapsing("Reconstruction details", |ui| {
                ui.label(&report.backend_reason);
                match &report.recognition_skipped {
                    Some(reason) => ui.label(format!(
                        "{} input triangles · whole-primitive recognition skipped: {reason}",
                        report.input_triangles
                    )),
                    None => ui.label(format!(
                        "{} input triangles · {} unresolved",
                        report.input_triangles, report.unresolved_triangles
                    )),
                };
                ui.label(format!(
                    "Completed in {:.2} seconds",
                    report.timings.total_seconds
                ));
                for message in &report.messages {
                    ui.label(message);
                }
            });
        }
        ui.separator();
        ui.checkbox(&mut self.engine.settings.wireframe, "Wireframe preview");
        if ui.button("Fit preview to view").clicked() {
            self.engine.zoom_to_fit();
        }
        action
    }

    pub fn state_json(&self) -> String {
        // The analytic tally the dialog SHOWS ("2 cylindrical · 0 conical · 0
        // spherical") counted from the same STEP text, so a reader outside the
        // window sees the same answer as a reader in front of it. It is the
        // whole point of a reconstruction — whether the walls of a bore came
        // back as cylinders or as a tessellation — and nothing published it.
        let surfaces = |kind: &str| {
            self.result
                .as_ref()
                .map(|(_, output)| output.step_text.matches(kind).count())
        };
        serde_json::json!({"name": self.name, "format": format_label(self.format),
            "ready": self.ready(), "busy": self.busy(),
            "revision": self.revision, "previewRevision": self.result.as_ref().map(|(r, _)| r),
            "error": self.error, "options": self.options, "solidCount": self.engine.scene.solids().len(),
            "faces": self.result.as_ref().map(|(_, r)| r.report.exported_advanced_faces),
            "cylindricalSurfaces": surfaces("CYLINDRICAL_SURFACE("),
            "conicalSurfaces": surfaces("CONICAL_SURFACE("),
            "sphericalSurfaces": surfaces("SPHERICAL_SURFACE(")
        }).to_string()
    }
    pub fn hits_json(&self) -> String {
        serde_json::to_string(&self.hits).unwrap()
    }
}

/// The format's name as the preview heading and `state_json` spell it.
fn format_label(format: MeshImportFormat) -> &'static str {
    match format {
        MeshImportFormat::Stl => "STL",
        MeshImportFormat::Obj => "OBJ",
        MeshImportFormat::ThreeMf => "3MF",
    }
}

fn number(
    ui: &mut egui::Ui,
    hits: &mut BTreeMap<String, [f32; 4]>,
    key: &str,
    label: &str,
    value: &mut f64,
    range: std::ops::RangeInclusive<f64>,
    speed: f64,
    help: &str,
) -> bool {
    ui.label(label).on_hover_text(help);
    let response = ui
        .add(
            egui::DragValue::new(value)
                .range(range)
                .speed(speed)
                .max_decimals(12),
        )
        .on_hover_text(help);
    let r = response.rect;
    // `field:<key>`, the prefix every other panel keys an editable value under
    // — and the one this panel's registry entry has always DOCUMENTED. The
    // rows published bare keys instead, so `hit_keys_check` called all six
    // undocumented the first time anything opened this dialog with it: the
    // browser verifier read the rect map directly and never asked.
    hits.insert(format!("field:{key}"), [r.min.x, r.min.y, r.width(), r.height()]);
    ui.end_row();
    response.changed()
}


/// The hit keys this panel publishes (see `automation::hit_keys`).
pub static HIT_KEYS: &[HitKeyDoc] = &[
    HitKeyDoc { panel: "importpreview", prefix: "accept", meaning: "accept the reconstructed import", command: None },
    HitKeyDoc { panel: "importpreview", prefix: "cancel", meaning: "cancel the import", command: None },
    HitKeyDoc { panel: "importpreview", prefix: "update", meaning: "update the preview with the current tolerances", command: None },
    HitKeyDoc { panel: "importpreview", prefix: "reset", meaning: "reset the tolerances", command: None },
    HitKeyDoc { panel: "importpreview", prefix: "strict", meaning: "require fully analytic surfaces", command: None },
    HitKeyDoc { panel: "importpreview", prefix: "autoWeld", meaning: "automatic vertex welding", command: None },
    HitKeyDoc { panel: "importpreview", prefix: "field:", meaning: "a tolerance field (field:distance, field:relative, field:normal, field:feature, field:fit, field:brepNormal)", command: None },
];
