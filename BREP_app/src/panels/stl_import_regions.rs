//! The interactive half of the mesh import preview: the mesh shown the moment
//! it is read, its regions coloured as the segmentation finds them, which of
//! them will fit (and as what) and which stay triangles, and the per-region
//! refit a user drives from a picked region.
//!
//! Everything here is display and intent. The regions, carriers, residuals and
//! refits come from the runner (`brep_reconstruction::import_session`); this
//! module never classifies a triangle itself.
use brep_render::brep_kernel;
use brep_render::engine_state::EngineState;
use brep_render::runner::{
    ApplyRegionChoicesJob, ConversionBackend, FitKind, ImportRegion, ImportRegions, MeshImportPhase, NurbsFaceChoice,
    NurbsFaceOutcome, RegionFit, RegionFitJob, RegionFitRequest, RegionKind, StlConversionReport, UNASSIGNED_REGION,
};
use brep_render::style::{FieldKind, FormField};
use eframe::egui;
use std::collections::{BTreeMap, BTreeSet, HashMap};

/// The display body's scene name.
pub const REGION_SOLID: &str = "Mesh regions";

/// What the viewport shows.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ViewMode {
    /// Each region its own colour.
    Regions,
    /// Coloured by what each region will become.
    FitStatus,
    /// The reconstructed BREP.
    Result,
}

impl ViewMode {
    fn id(self) -> &'static str {
        match self {
            Self::Regions => "regions",
            Self::FitStatus => "fit",
            Self::Result => "result",
        }
    }
}

/// The phase the import is in, as the list shows it.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Phase {
    Reading,
    Mesh,
    Regions,
    Built,
}

/// The region form's editable values.
#[derive(Clone, Debug)]
struct RegionForm {
    region: u32,
    kind: FitKind,
    tolerance: f64,
    normal_deg: f64,
}

#[derive(Default)]
pub struct RegionView {
    positions: Vec<f64>,
    indices: Vec<u32>,
    regions: Option<ImportRegions>,
    regions_error: Option<String>,
    /// The display body as built (faces in `face_region` order).
    display: Option<brep_render::scene::SolidDisplay>,
    /// Face index → region id ([`UNASSIGNED_REGION`] for the leftovers).
    face_region: Vec<u32>,
    view: Option<ViewMode>,
    /// The user picked the view; a landing result does not switch it.
    view_chosen: bool,
    selected: Option<u32>,
    form: Option<RegionForm>,
    /// The fit the user ran per region (the latest), and the one in flight.
    fits: BTreeMap<u32, RegionFit>,
    pending_fit: Option<(u64, u32)>,
    next_job: u64,
    /// The colours last written, to skip a no-op re-upload.
    painted: Vec<Option<[f32; 3]>>,
    /// The region list's filter: only the faceted regions.
    only_faceted: bool,
    /// What the BUILT body did with a proposed region the builder demoted:
    /// region id → why, in words (from the report's demotions).
    demoted: BTreeMap<u32, String>,
    /// The whole body was kept as triangles (or was not the mixed builder's,
    /// or held several bodies): why, and whether per-region outcomes apply.
    outcome_note: Option<String>,
    whole_faceted: bool,
    /// The camera was framed on the first mesh of this session.
    framed: bool,
    /// The Region field's pick is running: the next region face picked in
    /// the viewport fills it.
    picking: bool,
    /// Regions the user chose to rebuild as one NURBS face each (from an
    /// in-tolerance NURBS refit), applied by "Apply chosen fits".
    chosen: BTreeSet<u32>,
    /// "Apply chosen fits" was pressed: the preview sends the job with its STEP.
    apply_requested: bool,
    /// The job in flight and the regions it carries, in choice order.
    pending_apply: Option<(u64, Vec<u32>)>,
    /// What the last apply did with each region it carried.
    applied: BTreeMap<u32, NurbsFaceOutcome>,
    /// The last apply's error (the STEP could not be read or written).
    apply_error: Option<String>,
    /// The STEP with the applied faces, for the preview to show and accept.
    new_step: Option<String>,
}

/// A stable, well-spread colour per region id (golden-angle hues).
fn palette(id: u32) -> [f32; 3] {
    let hue = (id as f32 * 0.618_034).fract();
    let (s, v) = if id % 2 == 0 { (0.55, 0.92) } else { (0.70, 0.78) };
    hsv(hue, s, v)
}

fn hsv(h: f32, s: f32, v: f32) -> [f32; 3] {
    let i = (h * 6.0).floor();
    let f = h * 6.0 - i;
    let (p, q, t) = (v * (1.0 - s), v * (1.0 - f * s), v * (1.0 - (1.0 - f) * s));
    match i as i32 % 6 {
        0 => [v, t, p],
        1 => [q, v, p],
        2 => [p, v, t],
        3 => [p, q, v],
        4 => [t, p, v],
        _ => [v, p, q],
    }
}

/// The colour of a region's status in the Fit view.
fn kind_colour(kind: RegionKind) -> [f32; 3] {
    match kind {
        RegionKind::Plane => [0.36, 0.56, 0.90],
        RegionKind::Cylinder => [0.30, 0.74, 0.42],
        RegionKind::Cone => [0.95, 0.62, 0.22],
        RegionKind::Sphere => [0.66, 0.42, 0.86],
        RegionKind::Torus => [0.20, 0.72, 0.74],
        RegionKind::Nurbs => [0.92, 0.80, 0.25],
        RegionKind::Faceted => [0.88, 0.36, 0.36],
    }
}

const UNASSIGNED_COLOUR: [f32; 3] = [0.55, 0.55, 0.55];

fn egui_colour(c: [f32; 3]) -> egui::Color32 {
    egui::Color32::from_rgb((c[0] * 255.0) as u8, (c[1] * 255.0) as u8, (c[2] * 255.0) as u8)
}

fn short(value: f64) -> String {
    if value == 0.0 || !value.is_finite() {
        return format!("{value}");
    }
    let digits = (2 - value.abs().log10().floor() as i32).max(0) as usize;
    format!("{value:.digits$}")
}

impl RegionView {
    pub fn phase(&self, built: bool) -> Phase {
        if built {
            Phase::Built
        } else if self.regions.is_some() || self.regions_error.is_some() {
            Phase::Regions
        } else if self.display.is_some() {
            Phase::Mesh
        } else {
            Phase::Reading
        }
    }

    /// Forget everything about the last reconstruction (a new Update).
    pub fn restart(&mut self) {
        let view = self.view;
        let chosen = self.view_chosen;
        let framed = self.framed;
        // Job ids keep counting, so a reply to a job sent before the restart
        // never matches one sent after it.
        let next_job = self.next_job;
        *self = Self::default();
        self.framed = framed;
        self.next_job = next_job;
        if chosen {
            self.view = view;
            self.view_chosen = true;
        }
    }

    /// Read what the built body did with the proposed regions. The report's
    /// demotions are keyed by the builder's region ids, which are this
    /// segmentation's for a single body (the builder runs the same
    /// segmentation); a demotion whose region here has another kind is not
    /// claimed for it.
    pub fn apply_result(&mut self, report: &StlConversionReport) {
        self.demoted.clear();
        self.outcome_note = None;
        self.whole_faceted = false;
        if report.components.len() > 1 {
            self.outcome_note = Some(format!(
                "{} bodies: the region colours show the segmentation's proposal; per-region build outcomes are reported for single-body meshes",
                report.components.len()
            ));
            return;
        }
        match report.backend {
            ConversionBackend::FacetedRepair | ConversionBackend::FacetedRepairCoplanarMerged => {
                self.whole_faceted = true;
                self.outcome_note = Some(format!(
                    "No mixed shell could be closed, so the whole body was kept as repaired triangles{}",
                    report.fallback_cause.as_ref().map_or(String::new(), |cause| format!(": {cause}"))
                ));
            }
            ConversionBackend::HybridAnalyticRebuild => {
                for demotion in &report.hybrid_demotions {
                    let Some(region) = self.region(demotion.region) else { continue };
                    let kind = serde_json::to_value(demotion.kind).ok().and_then(|v| v.as_str().map(str::to_string)).unwrap_or_default();
                    if serde_json::to_value(region.kind).ok().and_then(|v| v.as_str().map(str::to_string)) != Some(kind) {
                        continue;
                    }
                    let words = |cause| serde_json::to_value(cause).ok().and_then(|v| v.as_str().map(|s| s.replace('_', " "))).unwrap_or_default();
                    let mut reason = format!("the builder demoted this {} to triangles: {}", region.kind.label(), words(demotion.cause));
                    if demotion.root_cause != demotion.cause {
                        reason.push_str(&format!(" (started by {})", words(demotion.root_cause)));
                    }
                    if let Some(neighbour) = demotion.neighbour {
                        reason.push_str(&format!(", at its edge with region #{neighbour}"));
                    }
                    if let Some(message) = &demotion.message {
                        reason.push_str(&format!(" — {message}"));
                    }
                    self.demoted.insert(demotion.region, reason);
                }
            }
            _ => {
                self.outcome_note = Some(format!("Built as a whole {} — every region is part of it", serde_json::to_value(report.backend).ok().and_then(|v| v.as_str().map(|s| s.replace('_', " "))).unwrap_or_default()));
            }
        }
    }

    /// Take one phase the runner posted. Returns whether the scene changed.
    pub fn take_phase(&mut self, phase: MeshImportPhase, engine: &mut EngineState) -> bool {
        match phase {
            MeshImportPhase::Mesh { positions, indices } => {
                self.positions = positions;
                self.indices = indices;
                self.rebuild_display();
                self.install(engine);
                // Frame the first mesh only: an Update keeps the user's view.
                if !self.framed {
                    engine.zoom_to_fit();
                    self.framed = true;
                }
                true
            }
            MeshImportPhase::Regions(regions) => {
                self.regions = Some(regions);
                self.rebuild_display();
                self.install(engine);
                true
            }
            MeshImportPhase::RegionsFailed(error) => {
                self.regions_error = Some(error);
                false
            }
        }
    }

    /// Build the display body: one face per region, triangles grouped by
    /// region, flat-shaded.
    fn rebuild_display(&mut self) {
        let triangle_count = self.indices.len() / 3;
        let labels: Vec<u32> = match &self.regions {
            Some(regions) if regions.triangle_regions.len() == triangle_count => regions.triangle_regions.clone(),
            _ => vec![UNASSIGNED_REGION; triangle_count],
        };
        let mut groups: BTreeMap<u32, Vec<usize>> = BTreeMap::new();
        for (t, &region) in labels.iter().enumerate() {
            groups.entry(region).or_default().push(t);
        }
        let mut mesh = brep_kernel::Mesh { positions: Vec::new(), normals: Vec::new(), indices: Vec::new(), face_ids: Vec::new() };
        let mut faces = Vec::new();
        self.face_region.clear();
        for (face, (region, triangles)) in groups.iter().enumerate() {
            let name = if *region == UNASSIGNED_REGION { "region:unassigned".to_string() } else { format!("region:{region}") };
            faces.push((face as u64 + 1, Some(name)));
            self.face_region.push(*region);
            for &t in triangles {
                let corner = |k: usize| {
                    let v = self.indices[3 * t + k] as usize;
                    [self.positions[3 * v], self.positions[3 * v + 1], self.positions[3 * v + 2]]
                };
                let [a, b, c] = [corner(0), corner(1), corner(2)];
                let u = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
                let w = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
                let n = [u[1] * w[2] - u[2] * w[1], u[2] * w[0] - u[0] * w[2], u[0] * w[1] - u[1] * w[0]];
                let length = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt().max(f64::MIN_POSITIVE);
                let n = [n[0] / length, n[1] / length, n[2] / length];
                for p in [a, b, c] {
                    mesh.positions.extend(p);
                    mesh.normals.extend(n);
                    mesh.indices.push(mesh.indices.len() as u32);
                }
                mesh.face_ids.push(face as u32);
            }
        }
        let payload = brep_kernel::DisplaySolidPayload { faces, mesh, edges: Vec::new(), vertices: Vec::new(), chord_tolerance: 0.0 };
        self.display = Some(brep_render::scene::solid_display_from_payload(REGION_SOLID, payload));
        self.painted.clear();
    }

    /// Put the display body in the engine's scene (a loaded result clears
    /// the scene, so this runs again after it).
    fn install(&mut self, engine: &mut EngineState) {
        if let Some(display) = &self.display {
            engine.scene.insert_solid(display.clone());
            self.painted.clear();
        }
    }

    /// The region record of `id`.
    fn region(&self, id: u32) -> Option<&ImportRegion> {
        self.regions.as_ref()?.regions.iter().find(|region| region.id == id)
    }

    /// What `id` is in the body Accept imports: the segmentation's proposal,
    /// unless the build demoted it (or kept the whole body as triangles).
    /// A refit is a CHECK for now — Accept does not apply it — so it is
    /// reported beside this ([`Self::refit_note`]), never as it.
    fn status(&self, id: u32) -> RegionKind {
        if self.applied.get(&id).is_some_and(|outcome| outcome.applied) {
            return RegionKind::Nurbs;
        }
        if self.whole_faceted || self.demoted.contains_key(&id) {
            return RegionKind::Faceted;
        }
        self.region(id).map_or(RegionKind::Faceted, |region| region.kind)
    }

    /// Why `id` stays triangles, when it does.
    fn reason(&self, id: u32) -> Option<String> {
        if let Some(outcome) = self.applied.get(&id) {
            if outcome.applied {
                return None;
            }
            if let Some(reason) = &outcome.reason {
                return Some(format!("the NURBS face was refused: {reason}"));
            }
        }
        if let Some(reason) = self.demoted.get(&id) {
            return Some(reason.clone());
        }
        if self.whole_faceted {
            return self.outcome_note.clone();
        }
        self.region(id).and_then(|region| region.reason.clone())
    }

    /// The user's refit of `id`, in a few words.
    fn refit_note(&self, id: u32) -> Option<String> {
        if let Some(outcome) = self.applied.get(&id).filter(|outcome| outcome.applied) {
            return Some(format!(
                "applied: one NURBS face{} (Accept imports it)",
                outcome.max_deviation.map_or(String::new(), |d| format!(" within {} mm", short(d)))
            ));
        }
        if self.chosen.contains(&id) {
            return Some("chosen: one NURBS face (Apply chosen fits)".into());
        }
        let fit = self.fits.get(&id)?;
        Some(match (fit.within_tolerance, fit.kind) {
            (true, Some(RegionKind::Nurbs)) if self.status(id) == RegionKind::Faceted => "refit: fits as a NURBS surface (Use this fit to apply it)".into(),
            (true, Some(kind)) => format!("refit: fits as {} (check only — Accept does not apply analytic refits yet)", kind.label()),
            _ => "refit: does not fit".to_string(),
        })
    }

    /// Whether region `id`'s latest refit can be chosen as its NURBS face: an
    /// in-tolerance NURBS fit of a region the body keeps as triangles.
    fn can_use_fit(&self, id: u32) -> bool {
        self.fits.get(&id).is_some_and(|fit| fit.within_tolerance && fit.kind == Some(RegionKind::Nurbs))
            && self.status(id) == RegionKind::Faceted
            && !self.applied.get(&id).is_some_and(|outcome| outcome.applied)
    }

    /// The apply job for the chosen regions over `step_text`, when "Apply
    /// chosen fits" was pressed (taken once).
    pub fn take_apply_job(&mut self, step_text: &str) -> Option<ApplyRegionChoicesJob> {
        if !std::mem::take(&mut self.apply_requested) || self.pending_apply.is_some() {
            return None;
        }
        let regions = self.regions.as_ref()?;
        let ids: Vec<u32> = self.chosen.iter().copied().collect();
        if ids.is_empty() {
            return None;
        }
        let choices = ids
            .iter()
            .map(|&id| NurbsFaceChoice {
                triangles: regions.triangles_of(id),
                tolerance: self.fits.get(&id).map_or(1e-3, |fit| fit.tolerance),
            })
            .collect();
        self.next_job += 1;
        let id = self.next_job;
        self.pending_apply = Some((id, ids));
        self.apply_error = None;
        Some(ApplyRegionChoicesJob { id, step_text: step_text.to_string(), positions: self.positions.clone(), indices: self.indices.clone(), choices })
    }

    /// The STEP the last apply wrote (taken once): the preview shows it and
    /// Accept imports it.
    pub fn take_new_step(&mut self) -> Option<String> {
        self.new_step.take()
    }

    /// How many regions the last apply made NURBS faces.
    pub fn applied_faces(&self) -> usize {
        self.applied.values().filter(|outcome| outcome.applied).count()
    }

    /// An apply is in flight (Accept waits for it).
    pub fn applying(&self) -> bool {
        self.pending_apply.is_some() || self.apply_requested
    }

    fn view(&self, built: bool) -> ViewMode {
        self.view.unwrap_or(if built { ViewMode::Result } else { ViewMode::Regions })
    }

    /// Per-frame scene upkeep: the display body present, coloured for the
    /// view, and shown or hidden against the result; a viewport pick of a
    /// region face selects it. Returns whether the scene changed.
    pub fn sync(&mut self, engine: &mut EngineState, built: bool) -> bool {
        let Some(display) = &self.display else { return false };
        let mut changed = false;
        if engine.scene.solid(REGION_SOLID).is_none() {
            engine.scene.insert_solid(display.clone());
            self.painted.clear();
            changed = true;
        }
        let view = self.view(built);
        let show_regions = view != ViewMode::Result || !built;
        let names: Vec<String> = engine.scene.solids().iter().map(|solid| solid.name.clone()).collect();
        for name in names {
            let visible = if name == REGION_SOLID { show_regions } else { !show_regions };
            if engine.scene.solid(&name).is_some_and(|solid| solid.visible != visible) {
                engine.scene.set_visible(&name, visible);
                changed = true;
            }
        }
        let colours: Vec<Option<[f32; 3]>> = self
            .face_region
            .iter()
            .map(|&region| {
                Some(if region == UNASSIGNED_REGION || self.regions.is_none() {
                    UNASSIGNED_COLOUR
                } else if view == ViewMode::FitStatus {
                    kind_colour(self.status(region))
                } else {
                    palette(region)
                })
            })
            .collect();
        if colours != self.painted {
            engine.scene.set_face_colors(REGION_SOLID, &colours);
            self.painted = colours;
            changed = true;
        }
        // A pick in the viewport selects its region.
        let picked = engine
            .emphasis
            .selected_faces
            .iter()
            .find_map(|name| name.strip_prefix("region:").and_then(|id| id.parse::<u32>().ok()));
        if let Some(id) = picked {
            if self.selected != Some(id) {
                self.select(Some(id), None);
                // A single-entity field closes on its one pick.
                self.picking = false;
            }
        }
        changed
    }

    /// Select region `id` (or none), filling the form with its tolerances.
    /// `engine`, when given, also selects its face in the viewport.
    fn select(&mut self, id: Option<u32>, engine: Option<&mut EngineState>) {
        self.selected = id;
        self.form = id.and_then(|id| {
            let region = self.region(id)?;
            let fit = self.fits.get(&id);
            Some(RegionForm {
                region: id,
                kind: fit.map_or(FitKind::Auto, |_| self.form.as_ref().filter(|f| f.region == id).map_or(FitKind::Auto, |f| f.kind)),
                tolerance: fit.map_or(region.fit_bar.max(1e-9), |fit| fit.tolerance),
                normal_deg: self.regions.as_ref().map_or(15.0, |r| r.normal_tolerance_deg),
            })
        });
        if let Some(engine) = engine {
            let faces: Vec<String> = id.map(|id| vec![format!("region:{id}")]).unwrap_or_default();
            engine.set_selection(&[], &faces, &[], &[]);
        }
    }

    /// Collect refits and applies the runner finished.
    pub fn pump(&mut self, engine: &mut EngineState) {
        while let Some(reply) = engine.take_apply_region_choices() {
            let Some((job, ids)) = self.pending_apply.take() else { continue };
            if reply.id != job {
                self.pending_apply = Some((job, ids));
                continue;
            }
            match reply.result {
                Ok((text, outcomes)) => {
                    for (id, outcome) in ids.into_iter().zip(outcomes) {
                        if outcome.applied {
                            self.chosen.remove(&id);
                        }
                        self.applied.insert(id, outcome);
                    }
                    self.new_step = Some(text);
                    self.painted.clear();
                }
                Err(error) => self.apply_error = Some(error),
            }
        }
        while let Some(reply) = engine.take_region_fit() {
            let Some((job, region)) = self.pending_fit else { continue };
            if reply.id != job {
                continue;
            }
            self.pending_fit = None;
            self.fits.insert(region, reply.fit);
        }
    }

    pub fn busy(&self) -> bool {
        self.pending_fit.is_some() || self.applying()
    }

    /// Send the selected region's refit to the runner.
    fn retry(&mut self, engine: &mut EngineState) {
        let Some(form) = self.form.clone() else { return };
        let Some(regions) = &self.regions else { return };
        // The region's own triangles as a small mesh: the runner keeps no
        // import state, and a region is a fraction of the whole.
        let mut map = HashMap::<u32, u32>::new();
        let mut positions = Vec::new();
        let mut indices = Vec::new();
        for (t, _) in regions.triangle_regions.iter().enumerate().filter(|(_, &r)| r == form.region) {
            for k in 0..3 {
                let v = self.indices[3 * t + k];
                let local = *map.entry(v).or_insert_with(|| {
                    positions.extend_from_slice(&self.positions[3 * v as usize..3 * v as usize + 3]);
                    (positions.len() / 3 - 1) as u32
                });
                indices.push(local);
            }
        }
        let triangles = (0..indices.len() as u32 / 3).collect();
        self.next_job += 1;
        let id = self.next_job;
        self.pending_fit = Some((id, form.region));
        engine.submit_region_fit(RegionFitJob {
            id,
            positions,
            indices,
            request: RegionFitRequest { triangles, kind: form.kind, tolerance: form.tolerance, normal_tolerance_deg: form.normal_deg },
        });
        self.pump(engine);
    }

    /// The interactive section of the preview panel.
    pub fn controls(
        &mut self,
        ui: &mut egui::Ui,
        engine: &mut EngineState,
        built: bool,
        reconstructing: bool,
        hits: &mut BTreeMap<String, [f32; 4]>,
    ) {
        let mut rects: HashMap<String, egui::Rect> = HashMap::new();
        ui.heading("Import phases");
        let phase = self.phase(built);
        let step = |ui: &mut egui::Ui, done: bool, active: bool, text: String| {
            ui.horizontal(|ui| {
                if done {
                    ui.label(egui::RichText::new("✔").color(egui::Color32::from_rgb(90, 190, 110)));
                } else if active {
                    ui.spinner();
                } else {
                    ui.weak("○");
                }
                ui.label(text);
            });
        };
        let triangles = self.indices.len() / 3;
        step(ui, phase >= Phase::Mesh, phase == Phase::Reading && reconstructing,
            if phase >= Phase::Mesh { format!("Mesh read: {triangles} triangles") } else { "Reading the mesh".into() });
        let region_text = match (&self.regions, &self.regions_error) {
            (Some(regions), _) => {
                let faceted = regions.regions.iter().filter(|r| r.kind == RegionKind::Faceted).count();
                format!("{} regions found · {} will fit · {} stay triangles", regions.regions.len(), regions.regions.len() - faceted, faceted)
            }
            (None, Some(error)) => format!("Regions unavailable: {error}"),
            (None, None) => "Grouping triangles into regions".into(),
        };
        step(ui, phase >= Phase::Regions, phase == Phase::Mesh && reconstructing, region_text);
        step(ui, built, phase >= Phase::Mesh && reconstructing && !built,
            if built { "BREP built".into() } else { "Fitting surfaces and building the BREP".into() });

        if self.display.is_none() {
            return;
        }
        ui.add_space(4.0);
        ui.horizontal(|ui| {
            ui.label("Show");
            let mut view = self.view(built);
            let mut choices = vec![(ViewMode::Regions, "regions", "Regions (one colour each)"), (ViewMode::FitStatus, "fit", "What each region fits as")];
            if built {
                choices.push((ViewMode::Result, "result", "Reconstructed BREP"));
            }
            let (changed, _) = crate::form::enum_dropdown(ui, "view", &choices, &mut view, Some(220.0), Some(&mut rects));
            if changed {
                self.view = Some(view);
                self.view_chosen = true;
            }
        });
        if self.view(built) == ViewMode::FitStatus {
            ui.horizontal_wrapped(|ui| {
                for kind in [RegionKind::Plane, RegionKind::Cylinder, RegionKind::Cone, RegionKind::Sphere, RegionKind::Torus, RegionKind::Nurbs, RegionKind::Faceted] {
                    ui.label(egui::RichText::new("■").color(egui_colour(kind_colour(kind))));
                    ui.label(kind.label());
                }
            });
        }

        // The region list: click a row to select it (or click its face in the
        // viewport).
        if self.regions.is_some() {
            let view = self.view(built);
            let rows: Vec<(u32, RegionKind, usize, bool)> = self
                .regions
                .iter()
                .flat_map(|regions| &regions.regions)
                .map(|region| (region.id, self.status(region.id), region.triangles, self.fits.get(&region.id).is_some_and(|fit| fit.within_tolerance)))
                .collect();
            ui.add_space(4.0);
            ui.horizontal(|ui| {
                ui.strong("Regions");
                let filter = ui.checkbox(&mut self.only_faceted, "only those left as triangles");
                rects.insert("regionsFaceted".into(), filter.rect);
            });
            let mut clicked = None;
            egui::ScrollArea::vertical().id_salt("import-region-list").max_height(220.0).show(ui, |ui| {
                for &(id, status, triangles, refit_fits) in &rows {
                    if self.only_faceted && status != RegionKind::Faceted {
                        continue;
                    }
                    let colour = if view == ViewMode::FitStatus { kind_colour(status) } else { palette(id) };
                    let text = format!(
                        "#{id}  {}{}  · {triangles} tri",
                        status.label(),
                        if refit_fits { " · refit fits" } else { "" },
                    );
                    let row = ui.horizontal(|ui| {
                        ui.label(egui::RichText::new("■").color(egui_colour(colour)));
                        ui.selectable_label(self.selected == Some(id), text)
                    });
                    rects.insert(format!("regionrow:{id}"), row.inner.rect);
                    if row.inner.clicked() {
                        clicked = Some(id);
                    }
                }
            });
            if let Some(id) = clicked {
                self.select(Some(id), Some(engine));
            }
        }

        if let Some(note) = &self.outcome_note {
            ui.weak(note);
        }
        // The chosen NURBS faces, applied to the body Accept imports.
        let applied = self.applied.values().filter(|outcome| outcome.applied).count();
        if !self.chosen.is_empty() || self.pending_apply.is_some() || !self.applied.is_empty() || self.apply_error.is_some() {
            ui.add_space(4.0);
            ui.strong("Fits applied on Accept");
            if self.pending_apply.is_some() {
                ui.horizontal(|ui| {
                    ui.spinner();
                    ui.label("Rebuilding the chosen regions as NURBS faces…");
                });
            } else if !self.chosen.is_empty() {
                let list: Vec<String> = self.chosen.iter().map(|id| format!("#{id}")).collect();
                ui.label(format!("Chosen as NURBS faces: {}", list.join(", ")));
                let button = ui.add_enabled(built && !reconstructing, egui::Button::new("Apply chosen fits"));
                rects.insert("regionsApply".into(), button.rect);
                if button.clicked() {
                    self.apply_requested = true;
                }
            }
            if applied > 0 {
                ui.label(format!("{applied} region(s) are NURBS faces in the body Accept imports (Reconstructed BREP view)."));
            }
            for (id, outcome) in self.applied.iter().filter(|(_, outcome)| !outcome.applied) {
                ui.colored_label(ui.visuals().warn_fg_color, format!("#{id}: {}", outcome.reason.as_deref().unwrap_or("refused")));
            }
            if let Some(error) = &self.apply_error {
                ui.colored_label(ui.visuals().error_fg_color, error);
            }
        }
        if let Some(form) = self.form.clone() {
            self.region_form(ui, engine, form, reconstructing, &mut rects);
        }
        for (key, r) in rects {
            hits.insert(key, [r.min.x, r.min.y, r.width(), r.height()]);
        }
    }

    /// The selected region's form: the standard form view over its kind and
    /// tolerances, with Retry.
    fn region_form(&mut self, ui: &mut egui::Ui, engine: &mut EngineState, form: RegionForm, reconstructing: bool, rects: &mut HashMap<String, egui::Rect>) {
        let Some(region) = self.region(form.region).cloned() else { return };
        let fields = vec![
            FormField { path: vec!["region".into()], label: "Region".into(), group: "Region".into(), kind: FieldKind::Reference { filter: vec!["face".into()], multiple: false } },
            FormField {
                path: vec!["kind".into()],
                label: "Fit as".into(),
                group: "Region".into(),
                kind: FieldKind::Enum { variants: FitKind::ALL.iter().map(|k| k.label().to_string()).collect() },
            },
            FormField { path: vec!["tolerance".into()], label: "Distance tolerance (mm)".into(), group: "Region".into(), kind: FieldKind::BoundedScalar { min: 1e-9, max: 1e6, step: 1e-4 } },
            FormField { path: vec!["normalAngle".into()], label: "Normal angle (°)".into(), group: "Region".into(), kind: FieldKind::BoundedScalar { min: 0.01, max: 89.0, step: 0.1 } },
            FormField { path: vec!["retry".into()], label: "Retry fit".into(), group: "Region".into(), kind: FieldKind::Button { label: "Retry fit".into() } },
        ];
        let mut fields = fields;
        let chosen = self.chosen.contains(&form.region);
        if chosen || self.can_use_fit(form.region) {
            let label = if chosen { "Do not use this fit" } else { "Use this NURBS fit" };
            fields.push(FormField { path: vec!["useFit".into()], label: label.into(), group: "Region".into(), kind: FieldKind::Button { label: label.into() } });
        }
        let mut params = serde_json::json!({
            "region": [format!("region:{}", form.region)],
            "kind": form.kind.label(),
            "tolerance": form.tolerance,
            "normalAngle": form.normal_deg,
        });
        let applied_face = self.applied.get(&form.region).filter(|outcome| outcome.applied);
        let proposal = match self.reason(form.region) {
            None if applied_face.is_some() => format!(
                "One NURBS face in the body Accept imports, within {} mm of the mesh.",
                applied_face.and_then(|outcome| outcome.max_deviation).map_or("—".into(), short)
            ),
            None => format!(
                "Fits as a {} within {} mm (bar {} mm).",
                region.kind.label(),
                short(region.max_deviation),
                short(region.fit_bar)
            ),
            Some(reason) => format!("Stays triangles: {reason}."),
        };
        let mut lines = Vec::new();
        if self.pending_fit.is_some_and(|(_, id)| id == form.region) {
            lines.push("Fitting…".to_string());
        } else if reconstructing {
            lines.push("Retry fit is available once the BREP is built.".to_string());
        } else if let Some(fit) = self.fits.get(&form.region) {
            lines.push(fit.message.clone());
            if let Some(outcome) = self.applied.get(&form.region).filter(|outcome| outcome.applied) {
                lines.push(format!(
                    "Applied: one NURBS face, {} control points, within {} mm of the mesh. Accept imports it.",
                    outcome.control_points.map_or("—".into(), |[u, v]| format!("{u} × {v}")),
                    outcome.max_deviation.map_or("—".into(), short)
                ));
            } else if chosen {
                lines.push("Chosen: Apply chosen fits rebuilds this region as one NURBS face of the body.".into());
            } else if self.can_use_fit(form.region) {
                lines.push("Use this NURBS fit, then Apply chosen fits: the region becomes one NURBS face of the body Accept imports.".into());
            } else if fit.within_tolerance {
                lines.push("A check for now: Accept does not apply analytic refits yet (only NURBS faces).".into());
            }
            for attempt in &fit.attempts {
                lines.push(format!(
                    "{}: {}{}",
                    attempt.kind.label(),
                    attempt.max_deviation.map_or("—".to_string(), |d| format!("{} mm", short(d))),
                    attempt.note.as_ref().map_or(String::new(), |note| format!(" ({note})"))
                ));
            }
        } else {
            lines.push("Choose a kind and tolerance, then Retry fit.".into());
        }
        let trailing = [("Fit result", lines)];
        let region_path = vec!["region".to_string()];
        let picked_names = vec![format!("region:{}", form.region)];
        let title = format!("Region #{}", form.region);
        let subtitle = format!("{} triangles · {:.3} mm²", region.triangles, region.area);
        let banner_colour = if self.reason(form.region).is_some() { ui.visuals().warn_fg_color } else { ui.visuals().text_color() };
        let spec = crate::form_view::FormViewSpec {
            title: &title,
            subtitle: Some(&subtitle),
            fields: &fields,
            hidden: None,
            banner: Some((&proposal, banner_colour)),
            trailing: Some(&trailing),
            extra: None,
            exit_label: "Close",
            rollback: false,
            hits_prefix: "regionform:",
            read_only: None,
            picking: self.picking.then_some(crate::form::ActivePick { path: &region_path, names: &picked_names }),
        };
        let out = crate::form_view::form_view(ui, &spec, &mut params, Some(rects));
        if out.changed {
            let kind = params["kind"].as_str().and_then(|label| FitKind::ALL.into_iter().find(|k| k.label() == label)).unwrap_or(form.kind);
            let number = |key: &str, fallback: f64| params[key].as_f64().or_else(|| params[key].as_str().and_then(|s| s.trim().parse().ok())).unwrap_or(fallback);
            if let Some(f) = self.form.as_mut() {
                f.kind = kind;
                f.tolerance = number("tolerance", form.tolerance);
                f.normal_deg = number("normalAngle", form.normal_deg);
            }
            // A removed reference line deselects.
            if params["region"].as_array().is_some_and(|names| names.is_empty()) {
                self.select(None, Some(engine));
                return;
            }
        }
        // The Region field's Select starts a viewport pick (`sync` ends it on
        // the next region face); Done ends it keeping the region.
        if out.ref_activate.is_some() {
            self.picking = true;
        }
        if out.pick_done {
            self.picking = false;
        }
        // The runner is one thread: a refit sent during the build would wait
        // for it, so Retry waits for the build instead.
        if out.button_clicked.as_deref() == Some("retry") && self.pending_fit.is_none() && !reconstructing {
            self.retry(engine);
        }
        if out.button_clicked.as_deref() == Some("useFit") && self.pending_apply.is_none() {
            if !self.chosen.remove(&form.region) {
                self.chosen.insert(form.region);
            }
        }
        if out.exit_clicked {
            self.picking = false;
            self.select(None, Some(engine));
        }
    }

    /// The state scripts read (`__brepImportRegions`).
    pub fn state_json(&self, built: bool) -> serde_json::Value {
        let regions = self.regions.as_ref().map(|regions| {
            regions
                .regions
                .iter()
                .map(|region| {
                    let fit = self.fits.get(&region.id);
                    serde_json::json!({
                        "id": region.id,
                        "triangles": region.triangles,
                        "proposed": region.kind,
                        "status": self.status(region.id),
                        "maxDeviation": region.max_deviation,
                        "fitBar": region.fit_bar,
                        "reason": self.reason(region.id),
                        "refit": self.refit_note(region.id),
                        "chosen": self.chosen.contains(&region.id),
                        "applied": self.applied.get(&region.id).map(|outcome| serde_json::json!({
                            "applied": outcome.applied, "maxDeviation": outcome.max_deviation,
                            "controlPoints": outcome.control_points, "reason": outcome.reason,
                        })),
                        "fit": fit.map(|fit| serde_json::json!({
                            "kind": fit.kind, "withinTolerance": fit.within_tolerance,
                            "maxDeviation": fit.max_deviation, "tolerance": fit.tolerance,
                            "message": fit.message,
                            "nurbs": fit.nurbs.as_ref().map(|n| serde_json::json!({"countU": n.count_u, "countV": n.count_v, "degree": n.degree})),
                        })),
                    })
                })
                .collect::<Vec<_>>()
        });
        serde_json::json!({
            "phase": match self.phase(built) { Phase::Reading => "reading", Phase::Mesh => "mesh", Phase::Regions => "regions", Phase::Built => "built" },
            "meshTriangles": self.indices.len() / 3,
            "view": self.view(built).id(),
            "regionsError": self.regions_error,
            "regions": regions,
            "selected": self.selected,
            "form": self.form.as_ref().map(|f| serde_json::json!({"region": f.region, "kind": f.kind, "tolerance": f.tolerance, "normalAngle": f.normal_deg})),
            "fitting": self.pending_fit.is_some(),
            "applying": self.applying(),
            "appliedNurbsFaces": self.applied.values().filter(|outcome| outcome.applied).count(),
            "applyError": self.apply_error,
            "picking": self.picking,
            "outcome": self.outcome_note,
            "demoted": self.demoted.len(),
        })
    }
}

