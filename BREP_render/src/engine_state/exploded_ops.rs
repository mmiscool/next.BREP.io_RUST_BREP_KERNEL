//! Assembly exploded views, built on the PMI explode rows (one mechanism, not
//! two): an exploded view IS a PMI view whose `explode` annotations are its
//! ordered STEPS. It is saved with the assembly in the `pmi` block, a drawing
//! sheet places it like any PMI view, and the PDF 3D export carries it.
//!
//! What this module adds on top:
//! * [`EngineState::exploded_views`]: the views that hold explode steps, for the
//!   Assembly workbench's list;
//! * [`EngineState::assembly_auto_explode`]: a new view with one step per
//!   top-level component, each moved straight away from the base component
//!   (the first fixed one, else the largest) by a distance proportional to the
//!   assembly's size, the outermost first, every step drawing its trail;
//! * playback: a PROGRESS from 0 (collapsed) to the number of steps (fully
//!   exploded) poses step `k` at `clamp(progress - k, 0, 1)` of its delta, on
//!   the displays and on the trails alike, and [`EngineState::exploded_tick`]
//!   moves it toward a target over time (Explode / Collapse). Progress is
//!   display state, never saved: a view activated afresh shows fully exploded.
use super::EngineState;
use brep_kernel::{PmiAnnotation, PmiAnnotationReport, PmiGeometry, PmiPose};
use serde_json::{json, Value};

/// The animation state of the active view's steps.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct ExplodePlayback {
    /// Steps shown, `0 ..= steps`; `None` = every step at full (the default).
    pub progress: Option<f64>,
    /// Where an animation is heading, and its speed in steps per second.
    pub target: Option<(f64, f64)>,
}

/// One step of an exploded view (an `explode` row of a PMI view).
#[derive(Debug, Clone, PartialEq)]
pub struct ExplodeStep {
    pub id: String,
    pub targets: Vec<String>,
    /// What a person reads for each target: a component's part name and id
    /// (`sub-a (ACOMP2)`), a solid's own name.
    pub labels: Vec<String>,
    pub translate: [f64; 3],
    pub trace: bool,
    pub enabled: bool,
}

/// A PMI view that holds explode steps.
#[derive(Debug, Clone, PartialEq)]
pub struct ExplodedView {
    pub id: String,
    pub name: String,
    pub steps: Vec<ExplodeStep>,
}

/// `geometry` with an explode delta taken `fraction` of the way: the
/// translation and rotation scale linearly, the scale toward 1. Other rows
/// pass through.
pub(crate) fn explode_at(geometry: &PmiGeometry, fraction: f64) -> PmiGeometry {
    match geometry {
        PmiGeometry::Explode { solids, translate, rotate_deg, scale, center, trace } => PmiGeometry::Explode {
            solids: solids.clone(),
            translate: translate.map(|v| v * fraction),
            rotate_deg: rotate_deg.map(|v| v * fraction),
            scale: scale.map(|v| 1.0 + (v - 1.0) * fraction),
            center: *center,
            trace: *trace,
        },
        other => other.clone(),
    }
}

/// Where the active view's playback shows what the full explode puts at a
/// point: the run lays every annotation out on the fully exploded bodies,
/// and a body part-way through its steps carries its annotations with it,
/// `shown ∘ full⁻¹` through that body's stages. Borrowed from the report and
/// asked per row, so a frame builds nothing for rows that do not move.
pub(crate) struct PlaybackPoses<'a> {
    progress: f64,
    /// The posed explode rows, in step order.
    steps: Vec<&'a PmiAnnotationReport>,
    /// Each posed explode row's step index.
    index: std::collections::HashMap<&'a str, usize>,
}

fn pose_of(geometry: &PmiGeometry) -> Option<PmiPose> {
    match geometry {
        PmiGeometry::Explode { translate, rotate_deg, scale, center, .. } => {
            Some(PmiPose { center: *center, translate: *translate, rotate_deg: *rotate_deg, scale: *scale })
        }
        _ => None,
    }
}

impl PlaybackPoses<'_> {
    /// Step `step`'s pose fully exploded and as shown now.
    fn stage(&self, step: usize) -> Option<(PmiPose, PmiPose)> {
        let geometry = &self.steps[step].geometry;
        let fraction = (self.progress - step as f64).clamp(0.0, 1.0);
        Some((pose_of(geometry)?, pose_of(&explode_at(geometry, fraction))?))
    }

    /// The stages that pose `solid`, in display order.
    fn stages_of(&self, solid: &str) -> Vec<(PmiPose, PmiPose)> {
        (0..self.steps.len())
            .filter(|&step| matches!(&self.steps[step].geometry, PmiGeometry::Explode { solids, .. } if solids.iter().any(|s| s == solid)))
            .filter_map(|step| self.stage(step))
            .collect()
    }

    /// The stages a row's points ride: an explode step's chip and gizmo its
    /// own step only (the run lays them at the step's own delta from its
    /// modeling centre), a balloon every stage of its target. Other rows are
    /// laid out on the modeling geometry and do not move.
    fn rider(&self, row: &PmiAnnotationReport) -> Vec<(PmiPose, PmiPose)> {
        match &row.geometry {
            PmiGeometry::Explode { .. } => self.index.get(row.id.as_str()).and_then(|&step| self.stage(step)).into_iter().collect(),
            PmiGeometry::Leader { balloon: true, anchor: Some(anchor), .. } => {
                // An edge head is on its own solid. A surface head is on
                // whichever member is nearest; members one step moves apart
                // from another are rare, and the first posed one stands in.
                let solid = match &anchor.mode {
                    brep_kernel::BalloonAnchorMode::Edge { solid, .. } => Some(solid.as_str()),
                    _ => anchor.solids.iter().map(String::as_str).find(|solid| !self.stages_of(solid).is_empty()),
                };
                solid.map(|solid| self.stages_of(solid)).unwrap_or_default()
            }
            _ => Vec::new(),
        }
    }

    /// Where `point`, laid out by the run for `row`, is shown now.
    pub(crate) fn shown(&self, row: &PmiAnnotationReport, point: [f64; 3]) -> [f64; 3] {
        let stages = self.rider(row);
        let model = stages.iter().rev().fold(point, |p, (full, _)| full.unapply(p));
        stages.iter().fold(model, |p, (_, shown)| shown.apply(p))
    }

    /// The inverse of [`Self::shown`]: a point placed on screen now, in the
    /// frame the run lays `row` out in (where a dragged label is stored).
    pub(crate) fn full(&self, row: &PmiAnnotationReport, point: [f64; 3]) -> [f64; 3] {
        let stages = self.rider(row);
        let model = stages.iter().rev().fold(point, |p, (_, shown)| shown.unapply(p));
        stages.iter().fold(model, |p, (full, _)| full.apply(p))
    }

    /// `row`'s geometry as shown now: a balloon's head rides its target.
    /// (An explode row's own trail follows [`explode_at`].)
    pub(crate) fn shown_geometry(&self, row: &PmiAnnotationReport) -> Option<PmiGeometry> {
        match &row.geometry {
            PmiGeometry::Leader { targets, dot, balloon: true, anchor } => Some(PmiGeometry::Leader {
                targets: targets.iter().map(|target| self.shown(row, *target)).collect(),
                dot: *dot,
                balloon: true,
                anchor: anchor.clone(),
            }),
            _ => None,
        }
    }
}

fn steps_of(annotations: &[PmiAnnotation], parts: &std::collections::HashMap<String, String>) -> Vec<ExplodeStep> {
    annotations
        .iter()
        .filter(|a| a.kind == "explode")
        .map(|a| {
            let position = &a.params["transform"]["position"];
            let translate = std::array::from_fn(|i| position[i].as_f64().unwrap_or(0.0));
            let targets = a.references("targets");
            let labels = targets
                .iter()
                .map(|t| parts.get(t).map_or_else(|| t.clone(), |part| format!("{part} ({t})")))
                .collect();
            ExplodeStep {
                id: a.params["id"].as_str().unwrap_or_default().to_string(),
                targets,
                labels,
                translate,
                trace: a.params["showTraceLine"].as_bool().unwrap_or(true),
                enabled: a.enabled,
            }
        })
        .collect()
}

impl EngineState {
    /// Every PMI view with at least one explode step, in view order.
    pub fn exploded_views(&self) -> Vec<ExplodedView> {
        let state = self.pmi_state();
        if !state.views.iter().any(|view| view.annotations.iter().any(|a| a.kind == "explode")) {
            return Vec::new();
        }
        // Each top-level component's part name, for the step labels (read per
        // feature: the panel asks every frame, so no whole-document copy).
        let mut parts: std::collections::HashMap<String, String> = (0..self.history.len())
            .filter(|&i| self.history.feature_type(i).as_deref() == Some("ACOMP"))
            .filter_map(|i| self.history.feature_params(i))
            .filter_map(|p| Some((p["id"].as_str()?.to_string(), p["partName"].as_str()?.to_string())))
            .collect();
        // A step on a placement inside a sub-assembly names that placement's part.
        for view in &state.views {
            for target in view.annotations.iter().filter(|a| a.kind == "explode").flat_map(|a| a.references("targets")) {
                if target.contains(':') && !parts.contains_key(&target) {
                    if let Some(part) = self.part_at_path(&target) {
                        parts.insert(target, part);
                    }
                }
            }
        }
        state
            .views
            .iter()
            .map(|view| ExplodedView { id: view.id.clone(), name: view.name.clone(), steps: steps_of(&view.annotations, &parts) })
            .filter(|view| !view.steps.is_empty())
            .collect()
    }

    /// Whether view `id` holds explode steps (cheap: no labels).
    pub fn is_exploded_view(&self, id: &str) -> bool {
        self.pmi_state().find_view(id).is_some_and(|view| view.annotations.iter().any(|a| a.kind == "explode"))
    }

    /// The number of explode steps the active view poses (enabled rows the
    /// last run resolved), the unit of [`ExplodePlayback::progress`].
    pub fn exploded_step_count(&self) -> usize {
        let Some(active) = self.pmi_active_view.as_deref() else { return 0 };
        self.pmi_report
            .as_ref()
            .and_then(|report| report.view(active))
            .map_or(0, |view| {
                view.annotations
                    .iter()
                    .filter(|row| row.enabled && row.status == brep_kernel::PmiStatus::Ok)
                    .filter(|row| matches!(row.geometry, PmiGeometry::Explode { .. }))
                    .count()
            })
    }

    /// How far step `index` (among the active view's posed explode rows) is
    /// shown, `0 ..= 1`.
    pub(crate) fn explode_fraction(&self, index: usize) -> f64 {
        match self.explode_playback.progress {
            None => 1.0,
            Some(progress) => (progress - index as f64).clamp(0.0, 1.0),
        }
    }

    /// The active view's playback poses, while it shows anything other than
    /// every step at full (`None` then: the run's layout is what is shown).
    pub(crate) fn explode_playback_poses(&self) -> Option<PlaybackPoses<'_>> {
        let progress = self.explode_playback.progress?;
        let active = self.pmi_active_view.as_deref()?;
        let view = self.pmi_report.as_ref()?.view(active)?;
        let steps: Vec<&PmiAnnotationReport> = view
            .annotations
            .iter()
            .filter(|row| row.enabled && row.status == brep_kernel::PmiStatus::Ok)
            .filter(|row| matches!(row.geometry, PmiGeometry::Explode { .. }))
            .collect();
        if progress >= steps.len() as f64 {
            return None;
        }
        let index = steps.iter().enumerate().map(|(step, row)| (row.id.as_str(), step)).collect();
        Some(PlaybackPoses { progress, steps, index })
    }

    /// The step index of explode row `id` among the active view's posed rows.
    pub(crate) fn explode_step_of(&self, id: &str) -> Option<usize> {
        let active = self.pmi_active_view.as_deref()?;
        self.pmi_report
            .as_ref()?
            .view(active)?
            .annotations
            .iter()
            .filter(|row| row.enabled && row.status == brep_kernel::PmiStatus::Ok)
            .filter(|row| matches!(row.geometry, PmiGeometry::Explode { .. }))
            .position(|row| row.id == id)
    }

    /// The playback state, for the panel and automation.
    /// The shown view's progress, `0 ..= steps` (the slider's value).
    pub fn exploded_progress(&self) -> f64 {
        self.explode_playback.progress.unwrap_or(self.exploded_step_count() as f64)
    }

    pub fn exploded_state_json(&self) -> Value {
        let steps = self.exploded_step_count();
        json!({
            "activeView": self.pmi_active_view,
            "steps": steps,
            "progress": self.explode_playback.progress.unwrap_or(steps as f64),
            "animating": self.explode_playback.target.is_some(),
            "views": self.exploded_views().iter().map(|view| json!({
                "id": view.id,
                "name": view.name,
                "steps": view.steps.iter().map(|step| json!({
                    "id": step.id, "targets": step.targets, "labels": step.labels, "translate": step.translate,
                    "trace": step.trace, "enabled": step.enabled,
                })).collect::<Vec<_>>(),
            })).collect::<Vec<_>>(),
        })
    }

    /// Show the active view's steps up to `progress` (clamped to
    /// `0 ..= steps`), stopping any animation.
    pub fn exploded_set_progress(&mut self, progress: f64) {
        self.explode_playback.target = None;
        self.set_explode_progress(progress);
    }

    fn set_explode_progress(&mut self, progress: f64) {
        let steps = self.exploded_step_count() as f64;
        self.explode_playback.progress = Some(progress.clamp(0.0, steps));
        if self.pmi_active_view.is_some() {
            self.pmi_apply_explode();
            self.refresh_pmi_overlay();
            // A step's gizmo rides its step.
            self.sync_transform_gizmo();
        }
    }

    /// Animate the active view: `explode` plays its steps out from where it
    /// stands to fully exploded, else back to collapsed, at `steps_per_second`.
    pub fn exploded_animate(&mut self, explode: bool, steps_per_second: f64) -> Result<(), String> {
        if self.pmi_active_view.is_none() {
            return Err("activate an exploded view first".into());
        }
        let steps = self.exploded_step_count() as f64;
        let to = if explode { steps } else { 0.0 };
        if self.explode_playback.progress.is_none() {
            // Fully exploded now: an Explode replays from collapsed.
            self.set_explode_progress(if explode { 0.0 } else { steps });
        }
        self.explode_playback.target = Some((to, steps_per_second.max(1e-3)));
        Ok(())
    }

    /// Advance an animation by `dt` seconds; `true` while another frame is
    /// needed. The app calls it once per frame.
    pub fn exploded_tick(&mut self, dt: f64) -> bool {
        let Some((to, speed)) = self.explode_playback.target else { return false };
        if self.pmi_active_view.is_none() {
            self.explode_playback.target = None;
            return false;
        }
        let from = self.explode_playback.progress.unwrap_or(to);
        let step = speed * dt.max(0.0);
        let next = if to > from { (from + step).min(to) } else { (from - step).max(to) };
        self.set_explode_progress(next);
        if (next - to).abs() < 1e-12 {
            self.explode_playback.target = None;
            return false;
        }
        true
    }

    /// The placements the selection covers, as occurrence paths, in selection
    /// order: for each selected solid, the shallowest placement above it whose
    /// every solid is selected. A whole sub-assembly picked in the viewport is
    /// `ACOMP1`; the pin inside it picked from the BOM is `ACOMP1:ACOMP2`. A
    /// body of the assembly's own (no placement) is not one.
    pub fn selected_occurrences(&self) -> Vec<String> {
        let selected: std::collections::HashSet<&str> = self.emphasis.selected_solids.iter().map(String::as_str).collect();
        let mut out: Vec<String> = Vec::new();
        for name in &self.emphasis.selected_solids {
            let mut path = String::new();
            for (at, _) in name.match_indices(':') {
                path = name[..at].to_string();
                let prefix = format!("{path}:");
                if self.scene.solids().iter().filter(|s| s.name.starts_with(&prefix)).all(|s| selected.contains(s.name.as_str())) {
                    break;
                }
            }
            if !path.is_empty() && !out.contains(&path) {
                out.push(path);
            }
        }
        out
    }

    /// Add a step to the shown exploded view that moves `targets` (the
    /// selection's placements when empty) straight up by their own size, and
    /// select it, so its gizmo is armed to drag it where it belongs. Returns
    /// the step's id.
    pub fn exploded_add_step(&mut self, targets: &[String]) -> Result<String, String> {
        let view = self
            .pmi_active_view
            .clone()
            .filter(|id| self.is_exploded_view(id))
            .ok_or("show an exploded view first")?;
        let targets: Vec<String> = if targets.is_empty() { self.selected_occurrences() } else { targets.to_vec() };
        if targets.is_empty() {
            return Err("select the components to move".into());
        }
        let mut low = [f64::INFINITY; 3];
        let mut high = [f64::NEG_INFINITY; 3];
        for target in &targets {
            let prefix = format!("{target}:");
            for solid in self.scene.solids().iter().filter(|s| s.name.starts_with(&prefix)) {
                for k in 0..3 {
                    low[k] = low[k].min(solid.bbox.min[k]);
                    high[k] = high[k].max(solid.bbox.max[k]);
                }
            }
        }
        if !low[0].is_finite() {
            return Err(format!("'{}' has no solids", targets.join(", ")));
        }
        let size = (0..3).map(|k| (high[k] - low[k]).powi(2)).sum::<f64>().sqrt().max(1.0);
        let mut state = self.pmi_state();
        let step = state.next_id("EXP");
        state.find_view_mut(&view).ok_or("the exploded view vanished")?.annotations.push(PmiAnnotation {
            plugin_replay: None,
            persistent_data: Value::Null,
            kind: "explode".into(),
            enabled: true,
            params: json!({
                "id": step,
                "targets": targets,
                "transform": { "position": [0.0, 0.0, size], "rotationEuler": [0, 0, 0], "scale": [1, 1, 1] },
                "showTraceLine": true,
            }),
            label_world: None,
        });
        self.write_pmi_state(state);
        self.explode_playback = ExplodePlayback::default();
        self.pmi_set_annotation_open(Some(&step));
        Ok(step)
    }

    /// A new exploded view of the assembly, one step per top-level component.
    /// The base component stays: the first fixed one, else the largest. Every
    /// other moves straight away from the base's centre (along +Z when the
    /// two centres coincide) by `spread` times half the assembly's diagonal,
    /// the outermost first. Returns the view's id; the view is active.
    pub fn assembly_auto_explode(&mut self, name: Option<&str>, spread: f64) -> Result<String, String> {
        self.assembly_auto_explode_deep(name, spread, 1)
    }

    /// The placements directly under `path` (the assembly's own for `None`),
    /// as `(occurrence path, isFixed)` in document order.
    fn placements_under(&self, path: Option<&str>) -> Vec<(String, bool)> {
        let of = |features: &[Value], prefix: &str| -> Vec<(String, bool)> {
            features
                .iter()
                .filter(|f| f["type"] == "ACOMP")
                .filter_map(|f| {
                    let id = f["inputParams"]["id"].as_str()?;
                    Some((format!("{prefix}{id}"), f["inputParams"]["isFixed"].as_bool().unwrap_or(false)))
                })
                .collect()
        };
        let Some(path) = path else {
            return of(self.history.features(), "");
        };
        let mut segments = path.split(':');
        let Some(top) = segments.next() else { return Vec::new() };
        let Some(params) = self.history.index_of(top).and_then(|i| self.history.feature_params(i)) else { return Vec::new() };
        let Some(mut part) = params["partName"].as_str().map(str::to_string) else { return Vec::new() };
        let mut library = self.history.parts_library();
        let mut document = &library[&part]["document"];
        for id in segments {
            let Some(next) = document["features"]
                .as_array()
                .and_then(|features| features.iter().find(|f| f["inputParams"]["id"].as_str() == Some(id)))
                .and_then(|f| f["inputParams"]["partName"].as_str())
            else {
                return Vec::new();
            };
            part = next.to_string();
            library = &document["partsLibrary"];
            document = &library[&part]["document"];
        }
        of(document["features"].as_array().map(Vec::as_slice).unwrap_or(&[]), &format!("{path}:"))
    }

    /// One group's steps: each placement but the base (the first fixed one,
    /// else the largest) moved straight away from the base's centre (+Z when
    /// the centres coincide) by `spread` × half the group's diagonal, the
    /// outermost first. `None` when fewer than two of them show a solid.
    fn explode_group(&self, placements: &[(String, bool)], spread: f64) -> Option<Vec<(String, f64, [f64; 3])>> {
        let boxes: Vec<(&str, bool, [f64; 3], [f64; 3])> = placements
            .iter()
            .filter_map(|(path, fixed)| {
                let prefix = format!("{path}:");
                let mut low = [f64::INFINITY; 3];
                let mut high = [f64::NEG_INFINITY; 3];
                for solid in self.scene.solids().iter().filter(|s| s.name.starts_with(&prefix)) {
                    for k in 0..3 {
                        low[k] = low[k].min(solid.bbox.min[k]);
                        high[k] = high[k].max(solid.bbox.max[k]);
                    }
                }
                low[0].is_finite().then_some((path.as_str(), *fixed, low, high))
            })
            .collect();
        if boxes.len() < 2 {
            return None;
        }
        let volume = |b: &(&str, bool, [f64; 3], [f64; 3])| (0..3).map(|k| (b.3[k] - b.2[k]).max(0.0)).product::<f64>();
        let base = boxes
            .iter()
            .position(|b| b.1)
            .unwrap_or_else(|| (0..boxes.len()).max_by(|&a, &b| volume(&boxes[a]).total_cmp(&volume(&boxes[b]))).unwrap_or(0));
        let centre = |b: &(&str, bool, [f64; 3], [f64; 3])| -> [f64; 3] { std::array::from_fn(|k| (b.2[k] + b.3[k]) / 2.0) };
        let origin = centre(&boxes[base]);
        let low: [f64; 3] = std::array::from_fn(|k| boxes.iter().map(|b| b.2[k]).fold(f64::INFINITY, f64::min));
        let high: [f64; 3] = std::array::from_fn(|k| boxes.iter().map(|b| b.3[k]).fold(f64::NEG_INFINITY, f64::max));
        let diagonal = (0..3).map(|k| (high[k] - low[k]).powi(2)).sum::<f64>().sqrt().max(1e-9);
        let distance = spread * diagonal / 2.0;
        let mut moves: Vec<(String, f64, [f64; 3])> = boxes
            .iter()
            .enumerate()
            .filter(|(i, _)| *i != base)
            .map(|(_, b)| {
                let c = centre(b);
                let d: [f64; 3] = std::array::from_fn(|k| c[k] - origin[k]);
                let length = d.iter().map(|v| v * v).sum::<f64>().sqrt();
                let direction = if length > 1e-6 * diagonal { d.map(|v| v / length) } else { [0.0, 0.0, 1.0] };
                (b.0.to_string(), length, direction.map(|v| v * distance))
            })
            .collect();
        // The outermost comes off first; a collapse brings it home last.
        moves.sort_by(|a, b| b.1.total_cmp(&a.1));
        Some(moves)
    }

    /// [`Self::assembly_auto_explode`] `depth` levels down: level 1 is the
    /// assembly's own placements; at each further level every sub-assembly
    /// placement (the base included) opens, its placements moving away from
    /// its own base by `spread` × its own size. A nested step moves on top of
    /// the steps of the placements above it. Steps go level by level, so
    /// Explode takes the big pieces apart first.
    pub fn assembly_auto_explode_deep(&mut self, name: Option<&str>, spread: f64, depth: usize) -> Result<String, String> {
        if !(spread.is_finite() && spread > 0.0) {
            return Err("the spread must be a positive number".into());
        }
        // Measure the assembly as placed: an active view's poses are put back first.
        self.pmi_deactivate_view();
        let mut moves: Vec<(String, f64, [f64; 3])> = Vec::new();
        let mut level: Vec<Option<String>> = vec![None];
        for _ in 0..depth.max(1) {
            let mut next = Vec::new();
            for parent in &level {
                let placements = self.placements_under(parent.as_deref());
                if let Some(group) = self.explode_group(&placements, spread) {
                    moves.extend(group);
                }
                next.extend(placements.into_iter().map(|(path, _)| Some(path)));
            }
            level = next;
        }
        if moves.is_empty() {
            return Err("an exploded view needs at least two placed components".into());
        }
        let name = name
            .map(str::trim)
            .filter(|n| !n.is_empty())
            .map(String::from)
            .unwrap_or_else(|| format!("Exploded view {}", self.exploded_views().len() + 1));
        // The view is framed on the EXPLODED layout: captured on the assembly
        // as it stands, the parts thrown furthest left the window (and so did
        // the gizmo that drags their steps). Each box at its exploded place,
        // the base where it is; the user's own camera is put back after the
        // capture (activating the view shows the view's).
        // Each solid at its exploded place: shifted by every step above it.
        let mut exploded = crate::camera::Aabb::empty();
        for solid in self.scene.solids().iter().filter(|s| s.visible) {
            let mut shift = [0.0; 3];
            for (path, _, translate) in &moves {
                if solid.name.starts_with(path.as_str()) && solid.name[path.len()..].starts_with(':') {
                    for k in 0..3 {
                        shift[k] += translate[k];
                    }
                }
            }
            exploded.expand(std::array::from_fn(|k| solid.bbox.min[k] + shift[k]));
            exploded.expand(std::array::from_fn(|k| solid.bbox.max[k] + shift[k]));
        }
        let camera = self.camera.clone();
        self.camera.zoom_to_fit(&exploded, 1.15);
        let id = self.pmi_capture_view(Some(&name));
        self.camera = camera;
        let mut state = self.pmi_state();
        let mut rows = Vec::new();
        for (component, _, translate) in moves {
            let step = state.next_id("EXP");
            rows.push(PmiAnnotation {
                plugin_replay: None,
                persistent_data: Value::Null,
                kind: "explode".into(),
                enabled: true,
                params: json!({
                    "id": step,
                    "targets": [component],
                    "transform": { "position": translate, "rotationEuler": [0, 0, 0], "scale": [1, 1, 1] },
                    "showTraceLine": true,
                }),
                label_world: None,
            });
        }
        state.find_view_mut(&id).ok_or("the new view vanished")?.annotations.extend(rows);
        self.write_pmi_state(state);
        self.explode_playback = ExplodePlayback::default();
        let _ = self.pmi_activate_view(&id);
        Ok(id)
    }
}

