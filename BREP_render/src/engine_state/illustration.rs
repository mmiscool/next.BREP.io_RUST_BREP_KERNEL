//! A presentation-only loan. No history edits or temporary documents are needed.
use super::*;
use serde_json::{json, Value};

#[derive(Clone)]
pub(super) struct IllustrationSnapshot {
    camera: crate::view::ViewCamera,
    settings: crate::style::RenderSettings,
    emphasis: crate::style::Emphasis,
    displays: Vec<crate::scene::SolidDisplay>,
    active_view: Option<String>,
    open_view: Option<String>,
    modeling: Option<super::pmi_ops::PmiModelingSnapshot>,
    explode: std::collections::HashMap<String, crate::scene::SolidDisplay>,
}

impl EngineState {
    pub fn illustration_begin(&mut self) -> Result<(), String> {
        if self.illustration_snapshot.is_some() {
            return Err("an illustration workflow is already active".into());
        }
        if self.sketch_edit.is_some() {
            return Err("finish sketch editing before an assembly illustration workflow".into());
        }
        if self.run_pending() {
            return Err("wait for the rebuild before capturing".into());
        }
        self.illustration_snapshot = Some(IllustrationSnapshot {
            camera: self.camera.clone(),
            settings: self.settings.clone(),
            emphasis: self.emphasis.clone(),
            displays: self.scene.solids().to_vec(),
            active_view: self.pmi_active_view.clone(),
            open_view: self.pmi_open_view.clone(),
            modeling: self.pmi_modeling.clone(),
            explode: self.pmi_explode_originals.clone(),
        });
        Ok(())
    }

    fn illustration_reset(&mut self) -> Result<(), String> {
        let snapshot = self
            .illustration_snapshot
            .clone()
            .ok_or("no active illustration workflow")?;
        self.pmi_restore_explode();
        for display in snapshot.displays {
            self.scene.set_visible(&display.name, display.visible);
            if let Some(live) = self.scene.solid_mut(&display.name) {
                *live = display;
            }
        }
        self.camera = snapshot.camera;
        self.settings = snapshot.settings;
        self.settings_generation += 1;
        self.widgets
            .set_viewcube_size(self.settings.viewcube_size_px);
        let generation = self.emphasis.generation;
        self.emphasis = snapshot.emphasis;
        self.emphasis.generation = generation + 1;
        self.pmi_active_view = snapshot.active_view;
        self.pmi_open_view = snapshot.open_view;
        self.pmi_modeling = snapshot.modeling;
        self.pmi_explode_originals = snapshot.explode;
        self.refresh_pmi_overlay();
        self.dirty = true;
        Ok(())
    }

    pub fn illustration_end(&mut self) -> Result<(), String> {
        self.illustration_reset()?;
        self.illustration_snapshot = None;
        Ok(())
    }

    /// Each requested view starts from the saved presentation, not its predecessor.
    pub fn illustration_apply(&mut self, view: &Value) -> Result<Value, String> {
        self.illustration_reset()?;
        let result = (|| {
            if let Some(id) = view["saved_view"].as_str() {
                self.pmi_activate_view(id)?;
            }
            let components: Vec<String> = if let Some(ids) = view.get("components") {
                serde_json::from_value(ids.clone()).map_err(|e| format!("components: {e}"))?
            } else {
                self.component_ids()
            };
            let mut solids = Vec::new();
            for id in &components {
                let info = self
                    .component_info(id)
                    .ok_or_else(|| format!("unknown component {id}"))?;
                if info.members.is_empty() {
                    return Err(format!("component {id} has no evaluated geometry"));
                }
                solids.extend(info.members);
            }
            if view.get("components").is_some() {
                let all: Vec<String> = self.scene.solids().iter().map(|s| s.name.clone()).collect();
                for name in all {
                    self.scene.set_visible(&name, solids.contains(&name));
                }
            }
            if let Some(visibility) = view.get("visibility") {
                for (id, visible) in visibility
                    .as_object()
                    .ok_or("visibility must map component ids to booleans")?
                {
                    let visible = visible
                        .as_bool()
                        .ok_or("visibility values must be boolean")?;
                    let info = self
                        .component_info(id)
                        .ok_or_else(|| format!("unknown component {id}"))?;
                    for name in info.members {
                        self.scene.set_visible(&name, visible);
                    }
                }
            }
            if let Some(explode) = view.get("explode") {
                for (id, value) in explode
                    .as_object()
                    .ok_or("explode must map component ids to translations in mm")?
                {
                    let translation: [f64; 3] = serde_json::from_value(value.clone())
                        .map_err(|e| format!("explode.{id}: {e}"))?;
                    if !translation.iter().all(|v| v.is_finite()) {
                        return Err("explode translation must be finite".into());
                    }
                    let info = self
                        .component_info(id)
                        .ok_or_else(|| format!("unknown component {id}"))?;
                    for name in info.members {
                        if let Some(display) = self.scene.solid_mut(&name) {
                            super::pmi_ops::transform_display(
                                display,
                                [0.0; 3],
                                translation,
                                [0.0; 3],
                                [1.0; 3],
                            );
                        }
                    }
                }
            }
            if let Some(settings) = view.get("settings") {
                if settings.get("lodFactor").is_some() {
                    return Err("capture settings cannot change lodFactor; set display LOD before starting the workflow".into());
                }
                self.apply_settings_json(&settings.to_string())?;
            }
            if view["clear_selection"].as_bool().unwrap_or(true) {
                let generation = self.emphasis.generation + 1;
                self.emphasis = Default::default();
                self.emphasis.generation = generation;
            }
            if let Some(camera) = view.get("camera") {
                self.apply_camera_state_json(&camera.to_string())?;
            } else if let Some(name) = view["standard_view"].as_str() {
                if !self.standard_view(&name.to_ascii_uppercase()) {
                    return Err(format!("unknown standard view {name}"));
                }
            }
            let visible_solids: Vec<&str> = self
                .scene
                .solids()
                .iter()
                .filter(|solid| solid.visible)
                .map(|solid| solid.name.as_str())
                .collect();
            let visible_components: Vec<String> = self
                .component_ids()
                .into_iter()
                .filter(|id| {
                    self.component_info(id).is_some_and(|info| {
                        info.members
                            .iter()
                            .any(|name| visible_solids.contains(&name.as_str()))
                    })
                })
                .collect();
            self.dirty = true;
            Ok(
                json!({"components":components, "solids":solids, "visibleComponents":visible_components, "visibleSolids":visible_solids, "camera":serde_json::from_str::<Value>(&self.camera_state_json()).unwrap_or_default()}),
            )
        })();
        if result.is_err() {
            self.illustration_reset()?;
        }
        result
    }
}

