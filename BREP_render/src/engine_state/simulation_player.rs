//! A motion study for the player: the baked frames as the player template's
//! `Motion` (per frame, the moved occurrences' pose deltas, the hidden ones,
//! opacities, the camera a camera clip drives, the clashing pairs), the
//! scene's meshes and nodes as the document poses them, and the offline
//! `.html` — the same trusted template the manufacturing instructions use
//! (`brep_instructions::player`), with no steps and a timeline instead.
//!
//! Nothing here needs a GPU: the web app makes the page too.

use super::simulation_play::{mat_mul, rigid_inverse, same_matrix, IDENTITY};
use super::EngineState;
use brep_instructions::model::Target;
use brep_instructions::player::{self, Camera, Mode, Motion, MotionFrame, PlayerData, PlayerMesh, PlayerNode, Scene};
use brep_kernel::{PmiCamera, PmiProjection};
use brep_simulation::model::BLOCK_KEY;
use serde_json::{json, Value};
use std::collections::BTreeMap;

/// The signature of an assembly document a published motion was made from:
/// the document's content hash with every study's published motion left out,
/// so it reads the same before and after a publish and differs once anything
/// else in the document (parts, mates, a study) changes.
pub fn assembly_signature(document: &Value) -> String {
    let mut stripped = document.clone();
    if let Some(studies) = stripped.get_mut(BLOCK_KEY).and_then(|block| block.get_mut("studies")).and_then(Value::as_array_mut) {
        for study in studies {
            if let Some(object) = study.as_object_mut() {
                object.remove("motion");
            }
        }
    }
    super::assembly_ops::document_signature(&stripped.to_string())
}

/// Pose deltas and opacities are written to this many decimals.
fn rounded(value: f64) -> f64 {
    (value * 1e4).round() / 1e4
}

/// A row-major 4×4 as the player multiplies it (column-major).
fn column_major(m: &[f64; 16]) -> [f64; 16] {
    std::array::from_fn(|k| {
        let (col, row) = (k / 4, k % 4);
        rounded(m[row * 4 + col])
    })
}

fn player_camera(camera: &PmiCamera) -> Camera {
    let (fov_deg, half_height) = match camera.projection {
        PmiProjection::Perspective { fov_y_deg } => (Some(fov_y_deg), None),
        PmiProjection::Orthographic { half_height } => (None, Some(half_height)),
    };
    Camera { eye: camera.eye, target: camera.target, up: camera.up, fov_deg, half_height }
}

impl EngineState {
    /// The study (the active one when `None`) baked to the end and read out
    /// frame by frame as the player's motion.
    pub fn simulation_motion(&mut self, study: Option<&str>) -> Result<Motion, String> {
        self.simulation_bake_all(study)?;
        self.motion_from_playback()
    }

    /// Publish the study (the active one when `None`): bake it to the end
    /// and write its motion into the document's `simulation` block as the
    /// study's `motion` — the player's `Motion` JSON with `study` and
    /// `assemblySignature` ([`assembly_signature`] of this document) beside
    /// the frames — for a manufacturing-instructions step to attach. One
    /// undo step; the bake is kept. Returns the study id and the frame count.
    pub fn simulation_publish(&mut self, study: Option<&str>) -> Result<(String, usize), String> {
        let id = self.simulation_bake_all(study)?;
        let motion = self.motion_from_playback()?;
        let frames = motion.frames.len();
        let document: Value = serde_json::from_str(&self.history.request_json()).map_err(|e| format!("the document does not read back: {e}"))?;
        let signature = assembly_signature(&document);
        let mut value = serde_json::to_value(&motion).expect("a motion serializes");
        value["study"] = json!(id);
        value["assemblySignature"] = json!(signature);
        let written = value.clone();
        self.simulation_edit(None, |block| {
            let study = block.study_mut(&id).ok_or_else(|| format!("no study '{id}'"))?;
            study.motion = Some(value);
            Ok(())
        })?;
        self.simulation_keep_bake_through_publish(&id, written);
        Ok((id, frames))
    }

    /// The player data of the study as its bake stands — no baking, no
    /// posing, nothing touched: what a PLM save publishes without disturbing
    /// the scene. `Err` until a study has been played or scrubbed and its
    /// bake has finished.
    pub fn simulation_player_data_baked(&self) -> Result<PlayerData, String> {
        let play = self.simulation_playback().ok_or("no study has been played or scrubbed yet")?;
        let total = play.baked_study().frame_count();
        if play.frames.len() < total {
            return Err(format!("the study's bake is not finished ({} of {total} frames): let it play once", play.frames.len()));
        }
        // A document edit since the bake (a mate, a part, a study, a PMI view
        // an explode clip plays) makes the frames another assembly's: the
        // next play re-bakes; until then nothing is published.
        if play.revision() != self.history.revision() {
            return Err("the study or the assembly changed since its bake: play it again".into());
        }
        self.player_data_from_playback()
    }

    /// The playback's frames as the player's motion (the bake as it stands).
    fn motion_from_playback(&self) -> Result<Motion, String> {
        let play = self.simulation_playback().ok_or("no study to publish")?;
        let record = play.baked_study().clone();
        let base = play.base().clone();
        let count = play.frames.len();
        let mut frames = Vec::with_capacity(count);
        for index in 0..count {
            let time = record.frame_time(index);
            let sample = record.sample(time, &|id| base.get(id).copied());
            let worlds = self.simulation_frame_worlds(index).unwrap_or_default();
            let mut poses = BTreeMap::new();
            for (occurrence, world) in &worlds {
                let Some(document) = self.occurrence_document_matrix(occurrence) else { continue };
                let delta = mat_mul(world, &rigid_inverse(&document));
                if !same_matrix(&delta, &IDENTITY) {
                    poses.insert(occurrence.clone(), column_major(&delta));
                }
            }
            let camera = sample.camera.as_ref().and_then(|c| self.simulation_camera_between(&c.from, &c.to, c.t)).map(|c| player_camera(&c));
            let mut clashing: Vec<String> = Vec::new();
            if let Some(frame) = self.simulation_playback().and_then(|play| play.frames.get(index)) {
                for (a, b) in &frame.clashes {
                    for id in [a, b] {
                        if !clashing.contains(id) {
                            clashing.push(id.clone());
                        }
                    }
                }
            }
            let opacity = sample.opacity.iter().map(|(id, value)| (id.clone(), rounded(*value))).collect();
            frames.push(MotionFrame { time: rounded(time), poses, hidden: sample.hidden.clone(), opacity, camera, clashing });
        }
        Ok(Motion { fps: record.fps, duration: record.duration, frames })
    }

    /// The study's player data: the scene's solids as the document poses them
    /// (a frame shown right now is read through its recorded originals), the
    /// study's motion, its name as the title.
    pub fn simulation_player_data(&mut self, study: Option<&str>) -> Result<PlayerData, String> {
        self.simulation_bake_all(study)?;
        self.player_data_from_playback()
    }

    /// [`Self::simulation_player_data`] over the bake as it stands.
    fn player_data_from_playback(&self) -> Result<PlayerData, String> {
        let motion = self.motion_from_playback()?;
        let play = self.simulation_playback().ok_or("no study to publish")?;
        let title = play.baked_study().name.clone();
        let mut scene = Scene::default();
        let flat = |values: &[[f32; 3]]| values.iter().flatten().copied().collect::<Vec<f32>>();
        for solid in self.scene.solids() {
            let display = play.original(&solid.name).unwrap_or(solid);
            if display.is_sketch || display.mesh.indices.is_empty() {
                continue;
            }
            let color = display.color_override.unwrap_or_else(|| crate::color::solid_color_srgb(&display.name).map(|c| c as f32));
            scene.nodes.push(PlayerNode { occurrence: solid.name.clone(), mesh: scene.meshes.len(), matrix: IDENTITY, color });
            scene.meshes.push(PlayerMesh::new(&solid.name, &flat(&display.mesh.positions), &flat(&display.mesh.normals), &display.mesh.indices));
        }
        if scene.nodes.is_empty() {
            return Err("nothing to show: the assembly has no solids".into());
        }
        PlayerData::motion_study(&title, "", Target::default(), scene, motion)
    }

    /// The study's offline player: one self-contained `.html` with the data
    /// inlined. Returns the page and the data it carries.
    pub fn simulation_html(&mut self, study: Option<&str>) -> Result<(String, PlayerData), String> {
        let data = self.simulation_player_data(study)?;
        Ok((player::offline_html(&data, Mode::Customer), data))
    }
}
