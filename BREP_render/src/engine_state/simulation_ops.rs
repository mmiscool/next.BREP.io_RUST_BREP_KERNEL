//! Motion studies on the engine: an assembly document keeps its studies in
//! its own `simulation` block (`brep_simulation::model`), and a study drives
//! that document's mates.
//!
//! What this module adds on top of the crate:
//! * [`EngineState::simulation`] / [`EngineState::simulation_edit`]: read the
//!   block, and read-modify-write it as one undo step (the block is started
//!   on the first edit);
//! * [`EngineState::simulation_motor_mates`]: the mates a motor can drive
//!   (this document's distance and angle constraints) with their values;
//! * [`EngineState::simulation_patch`]: merge fields into a study, motor or
//!   clip, so every editor (pane, automation) writes through one door.

use super::EngineState;
use brep_simulation::model::{Simulation, BLOCK_KEY};
use serde_json::{Map, Value};

/// A mate a motor can drive.
#[derive(Debug, Clone, PartialEq, serde::Serialize)]
#[serde(rename_all = "camelCase")]
pub struct MotorMate {
    /// The constraint id (`DIST2`).
    pub id: String,
    /// `distance` or `angle`.
    pub kind: String,
    /// Its document value (model units or degrees); a mate that adopts its
    /// value on first solve and has not solved yet reads 0.
    pub value: f64,
    pub enabled: bool,
}

impl MotorMate {
    /// The unit a motor on it runs in, per second.
    pub fn rate_unit(&self) -> &'static str {
        if self.kind == "angle" { "°/s" } else { "mm/s" }
    }
}

impl EngineState {
    /// The document's simulation block; `Ok(None)` when it has none.
    pub fn simulation(&self) -> Result<Option<Simulation>, String> {
        match self.history.document_block(BLOCK_KEY) {
            None | Some(Value::Null) => Ok(None),
            Some(block) => Simulation::from_value(block).map(Some),
        }
    }

    /// Whether the document has an assembly whose mates a study can drive.
    pub fn simulation_available(&self) -> bool {
        self.history_has_assembly()
    }

    /// Read the block (a new one when the document has none), apply `edit`,
    /// validate and write it back as ONE undo step (`coalesce` groups a typing
    /// run). Nothing is written on a refusal.
    pub fn simulation_edit<T>(
        &mut self,
        coalesce: Option<&str>,
        edit: impl FnOnce(&mut Simulation) -> Result<T, String>,
    ) -> Result<T, String> {
        if !self.simulation_available() {
            return Err("open an assembly: a study drives its mates".into());
        }
        let mut block = self.simulation()?.unwrap_or_default();
        let out = edit(&mut block)?;
        block.validate()?;
        self.history.set_document_block(BLOCK_KEY, Some(block.to_value()), coalesce);
        Ok(out)
    }

    /// This document's distance and angle constraints, in history order.
    pub fn simulation_motor_mates(&self) -> Vec<MotorMate> {
        let constraints = self.history.document_block("assembly").and_then(|a| a.get("constraints")).and_then(Value::as_array);
        constraints
            .into_iter()
            .flatten()
            .filter_map(|entry| {
                let kind = entry.get("type")?.as_str()?;
                let key = match kind {
                    "distance" => "distance",
                    "angle" => "angle",
                    _ => return None,
                };
                let params = entry.get("inputParams")?;
                Some(MotorMate {
                    id: params.get("id")?.as_str()?.to_string(),
                    kind: kind.to_string(),
                    value: params.get(key).and_then(Value::as_f64).unwrap_or(0.0),
                    enabled: entry.get("enabled").and_then(Value::as_bool).unwrap_or(true),
                })
            })
            .collect()
    }

    /// A new study (numbered when `name` is empty), made active. One undo step.
    pub fn simulation_add_study(&mut self, name: &str) -> Result<String, String> {
        self.simulation_edit(None, |block| Ok(block.add_study(name)))
    }

    /// Make `study` the one the workbench shows.
    pub fn simulation_set_active(&mut self, study: &str) -> Result<(), String> {
        self.simulation_edit(None, |block| {
            block.study(study).ok_or_else(|| format!("no study '{study}'"))?;
            block.active = Some(study.to_string());
            Ok(())
        })
    }

    /// Add a motor on mate `constraint` (the first mate a motor can drive
    /// when `None`) to `study` (the active one when `None`): the whole study
    /// long at 90 °/s for an angle mate, 10 mm/s for a distance mate. Returns
    /// `(study id, motor id)`.
    pub fn simulation_add_motor(&mut self, study: Option<&str>, constraint: Option<&str>) -> Result<(String, String), String> {
        let mates = self.simulation_motor_mates();
        let mate = match constraint {
            Some(id) => mates.iter().find(|m| m.id == id).ok_or_else(|| format!("'{id}' is not a distance or angle mate of this assembly"))?,
            None => mates.first().ok_or("add a distance or angle mate first: a motor drives one")?,
        };
        let speed = if mate.kind == "angle" { 90.0 } else { 10.0 };
        let constraint = mate.id.clone();
        self.simulation_edit(None, |block| {
            let study_id = match study {
                Some(id) => id.to_string(),
                None => match block.active_study() {
                    Some(active) => active.id.clone(),
                    None => block.add_study(""),
                },
            };
            let id = block.next_id("MOT");
            let target = block.study_mut(&study_id).ok_or_else(|| format!("no study '{study_id}'"))?;
            let to = target.duration;
            target.motors.push(brep_simulation::Motor { id: id.clone(), constraint, from: 0.0, to, drive: brep_simulation::Drive::Speed { speed } });
            Ok((study_id, id))
        })
    }

    /// Every constraint of the assembly block, by id, for a suppress clip to
    /// pick from (a motor drives only distance and angle mates; a suppress
    /// clip switches any mate off).
    pub fn simulation_constraint_ids(&self) -> Vec<String> {
        self.history
            .document_block("assembly")
            .and_then(|a| a.get("constraints"))
            .and_then(Value::as_array)
            .map(|rows| rows.iter().filter_map(|row| row.get("inputParams")?.get("id")?.as_str().map(str::to_string)).collect())
            .unwrap_or_default()
    }

    /// Add a clip of `kind` to `study` (the active one when `None`; a study
    /// is started when there is none), on `target` or the natural default:
    /// `suppress` the first constraint, `camera` through every PMI view (or
    /// the one named), `visibility` or `transparency` (1 → 0.3) of the
    /// selected occurrences or the first component, `section` of the first
    /// PMI view with a section, `explode` of the first exploded view. The
    /// span is the whole study. Returns `(study id, clip id)`. One undo step.
    pub fn simulation_add_clip(&mut self, study: Option<&str>, kind: &str, target: Option<&str>) -> Result<(String, String), String> {
        use brep_simulation::ClipBody;
        let body = match kind {
            "suppress" => {
                let ids = self.simulation_constraint_ids();
                let constraint = match target {
                    Some(id) if ids.iter().any(|c| c == id) => id.to_string(),
                    Some(id) => return Err(format!("'{id}' is not a constraint of this assembly")),
                    None => ids.first().cloned().ok_or("add a mate first: a suppress clip switches one off")?,
                };
                ClipBody::Suppress { constraint }
            }
            "camera" => {
                let views: Vec<String> = self.pmi_state().views.iter().filter(|v| v.camera.is_some()).map(|v| v.id.clone()).collect();
                let views = match target {
                    Some(id) if views.iter().any(|v| v == id) => vec![id.to_string()],
                    Some(id) => return Err(format!("'{id}' is not a PMI view with a camera")),
                    None if views.is_empty() => return Err("capture a PMI view first: a camera clip moves between views".into()),
                    None => views,
                };
                ClipBody::Camera { views }
            }
            "visibility" => {
                let occurrences = match target {
                    Some(id) => vec![id.to_string()],
                    None => {
                        let selected = self.selected_occurrences();
                        if selected.is_empty() {
                            vec![self.component_ids().first().cloned().ok_or("add a component first: a visibility clip hides one")?]
                        } else {
                            selected
                        }
                    }
                };
                ClipBody::Visibility { occurrences }
            }
            "explode" => {
                let views = self.exploded_views();
                let view = match target {
                    Some(id) if views.iter().any(|v| v.id == id) => id.to_string(),
                    Some(id) => return Err(format!("'{id}' is not an exploded view")),
                    None => views.first().map(|v| v.id.clone()).ok_or("make an exploded view first (Assembly: Auto explode): an explode clip plays its steps")?,
                };
                ClipBody::Explode { view, collapse: false }
            }
            "transparency" => {
                let occurrences = match target {
                    Some(id) => vec![id.to_string()],
                    None => {
                        let selected = self.selected_occurrences();
                        if selected.is_empty() {
                            vec![self.component_ids().first().cloned().ok_or("add a component first: a transparency clip fades one")?]
                        } else {
                            selected
                        }
                    }
                };
                ClipBody::Transparency { occurrences, opacity: [1.0, 0.3] }
            }
            "section" => {
                let views: Vec<String> = self.pmi_state().views.iter().filter(|v| v.display.section.is_some()).map(|v| v.id.clone()).collect();
                let view = match target {
                    Some(id) if views.iter().any(|v| v == id) => id.to_string(),
                    Some(id) => return Err(format!("'{id}' is not a PMI view with a section")),
                    None => views.first().cloned().ok_or("give a PMI view a section first: a section clip plays it")?,
                };
                ClipBody::Section { view, sweep: None }
            }
            other => return Err(format!("no clip kind '{other}': suppress, camera, visibility, transparency, section or explode")),
        };
        self.simulation_edit(None, |block| {
            let study_id = match study {
                Some(id) => id.to_string(),
                None => match block.active_study() {
                    Some(active) => active.id.clone(),
                    None => block.add_study(""),
                },
            };
            let id = block.next_id("CLIP");
            let target = block.study_mut(&study_id).ok_or_else(|| format!("no study '{study_id}'"))?;
            let to = target.duration;
            target.clips.push(brep_simulation::Clip { id: id.clone(), from: 0.0, to, body });
            Ok((study_id, id))
        })
    }

    /// Delete the study `study`, or its motor or clip `item`. One undo step.
    pub fn simulation_remove(&mut self, study: &str, item: Option<&str>) -> Result<(), String> {
        self.simulation_edit(None, |block| {
            match item {
                None => {
                    let before = block.studies.len();
                    block.studies.retain(|s| s.id != study);
                    if block.studies.len() == before {
                        return Err(format!("no study '{study}'"));
                    }
                    if block.active.as_deref() == Some(study) {
                        block.active = block.studies.first().map(|s| s.id.clone());
                    }
                }
                Some(id) => {
                    let target = block.study_mut(study).ok_or_else(|| format!("no study '{study}'"))?;
                    let before = target.motors.len() + target.clips.len();
                    target.motors.retain(|m| m.id != id);
                    target.clips.retain(|c| c.id != id);
                    if target.motors.len() + target.clips.len() == before {
                        return Err(format!("study '{study}' has no motor or clip '{id}'"));
                    }
                }
            }
            Ok(())
        })
    }

    /// What automation and the pane read: the block (or `null`), the mates a
    /// motor can drive, and whether the document can hold a study.
    pub fn simulation_state_json(&self) -> Value {
        let block = match self.simulation() {
            Ok(block) => block.map(|b| b.to_value()).unwrap_or(Value::Null),
            Err(error) => serde_json::json!({ "error": error }),
        };
        serde_json::json!({
            "available": self.simulation_available(),
            "block": block,
            "mates": self.simulation_motor_mates(),
        })
    }

    /// Merge `patch`'s fields into the study `study`, or into its motor or
    /// clip `item` (an object patch; `null` removes an optional field), as one
    /// undo step. The result must still be a valid study.
    pub fn simulation_patch(&mut self, study: &str, item: Option<&str>, patch: &Value, coalesce: Option<&str>) -> Result<(), String> {
        let patch = patch.as_object().ok_or("a patch is an object of fields")?.clone();
        self.simulation_edit(coalesce, |block| {
            let target = block.study_mut(study).ok_or_else(|| format!("no study '{study}'"))?;
            match item {
                None => {
                    if patch.contains_key("id") || patch.contains_key("motors") || patch.contains_key("clips") {
                        return Err("a study patch edits its own fields; motors and clips are edited one by one".into());
                    }
                    *target = merged(&*target, &patch)?;
                }
                Some(id) => {
                    if patch.contains_key("id") {
                        return Err("an id cannot be patched".into());
                    }
                    if let Some(motor) = target.motors.iter_mut().find(|m| m.id == id) {
                        *motor = merged(&*motor, &patch)?;
                    } else if let Some(clip) = target.clips.iter_mut().find(|c| c.id == id) {
                        *clip = merged(&*clip, &patch)?;
                    } else {
                        return Err(format!("study '{study}' has no motor or clip '{id}'"));
                    }
                }
            }
            Ok(())
        })
    }
}

/// `value` with `patch`'s top-level fields merged in, read back as a `T`.
fn merged<T: serde::Serialize + serde::de::DeserializeOwned>(value: &T, patch: &Map<String, Value>) -> Result<T, String> {
    let mut object = serde_json::to_value(value).map_err(|e| e.to_string())?;
    let fields = object.as_object_mut().ok_or("not an object")?;
    for (key, field) in patch {
        if field.is_null() {
            fields.remove(key);
        } else {
            fields.insert(key.clone(), field.clone());
        }
    }
    serde_json::from_value(object).map_err(|e| format!("that edit does not make a valid study: {e}"))
}
