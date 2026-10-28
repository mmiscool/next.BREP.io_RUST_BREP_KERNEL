//! A motion study's published motion on a manufacturing-instructions step.
//!
//! The assembly the instructions describe is placed as the grounded wrapper
//! with its document embedded ([`EngineState::instructions_level`] reads it);
//! a study the Simulation workbench published there carries its `motion`
//! (the player's `Motion` JSON with `study` and `assemblySignature` beside
//! the frames). A step takes a copy of it as its own `motion`
//! (`edit::update_step`), so the instructions play without the assembly at
//! hand and the player ([`EngineState::instructions_player_data`]) re-keys it
//! under the wrapper. A motion published before the assembly changed is
//! refused: its signature no longer matches the embedded document's.

use super::simulation_player::assembly_signature;
use super::EngineState;
use brep_instructions::edit;
use brep_simulation::model::{Simulation, BLOCK_KEY};
use serde_json::{json, Value};

/// A study of the assembly whose published motion a step can take.
#[derive(Debug, Clone, PartialEq, serde::Serialize)]
#[serde(rename_all = "camelCase")]
pub struct PublishedMotion {
    pub study: String,
    pub name: String,
    pub frames: usize,
    pub duration: f64,
    pub fps: u32,
    /// Whether the motion was published from the assembly as the
    /// instructions hold it (a stale one cannot be attached).
    pub current: bool,
}

impl EngineState {
    /// The wrapper's embedded document and its signature.
    fn instructions_target_document(&self) -> Result<(Value, String), String> {
        let (_, part) = self.instructions_wrapper()?;
        let document = self.history.parts_library()[part.as_str()]["document"].clone();
        if document.is_null() {
            return Err(format!("'{part}' has no embedded document"));
        }
        let signature = assembly_signature(&document);
        Ok((document, signature))
    }

    /// The assembly's studies that carry a published motion, in block order.
    pub fn instructions_published_motions(&self) -> Result<Vec<PublishedMotion>, String> {
        let (document, signature) = self.instructions_target_document()?;
        let block = match document.get(BLOCK_KEY) {
            None | Some(Value::Null) => return Ok(Vec::new()),
            Some(block) => Simulation::from_value(block)?,
        };
        Ok(block
            .studies
            .iter()
            .filter_map(|study| {
                let motion = study.motion.as_ref()?;
                Some(PublishedMotion {
                    study: study.id.clone(),
                    name: study.name.clone(),
                    frames: motion.get("frames").and_then(Value::as_array).map_or(0, Vec::len),
                    duration: motion.get("duration").and_then(Value::as_f64).unwrap_or(study.duration),
                    fps: motion.get("fps").and_then(Value::as_u64).map_or(study.fps, |f| f as u32),
                    current: motion.get("assemblySignature").and_then(Value::as_str) == Some(signature.as_str()),
                })
            })
            .collect())
    }

    /// Attach the published motion of `study` (the only published one when
    /// `None`) to `step` as its motion. Refused when the study has none, when
    /// it was published before the assembly changed (publish it again there),
    /// or when several are published and none is named. One undo step.
    /// Returns the study id.
    pub fn instructions_attach_motion(&mut self, step: &str, study: Option<&str>) -> Result<String, String> {
        let (document, signature) = self.instructions_target_document()?;
        let block = match document.get(BLOCK_KEY) {
            None | Some(Value::Null) => return Err("the assembly has no motion study: publish one in the Simulation workbench".into()),
            Some(block) => Simulation::from_value(block)?,
        };
        let published: Vec<_> = block.studies.iter().filter(|s| s.motion.is_some()).collect();
        let study = match study {
            Some(id) => block.study(id).ok_or_else(|| format!("the assembly has no study '{id}'"))?,
            None => match published.as_slice() {
                [one] => one,
                [] => return Err("the assembly has no published motion: publish a study in the Simulation workbench".into()),
                _ => return Err(format!("the assembly has {} published motions: name the study", published.len())),
            },
        };
        let motion = study.motion.as_ref().ok_or_else(|| format!("study '{}' is not published: publish it in the Simulation workbench", study.name))?;
        if motion.get("assemblySignature").and_then(Value::as_str) != Some(signature.as_str()) {
            return Err(format!("study '{}' was published before the assembly changed: publish it again there", study.name));
        }
        if motion.get("frames").and_then(Value::as_array).is_none_or(Vec::is_empty) {
            return Err(format!("study '{}' published no frames", study.name));
        }
        let id = study.id.clone();
        let patch = json!({ "motion": motion });
        self.instructions_edit(None, |block| edit::update_step(block, step, &patch))?;
        Ok(id)
    }

    /// Take the motion off `step`; the insertion animation plays again.
    pub fn instructions_detach_motion(&mut self, step: &str) -> Result<(), String> {
        self.instructions_edit(None, |block| edit::update_step(block, step, &json!({ "motion": null })))
    }
}
