//! Motion-study playback: bake, play, scrub.
//!
//! A study's frames are solved ONCE, in order, through the kernel's frame
//! solve (`assembly_frame_solve_impl`: the mates at the motors' values and the
//! suppress clips' switches, warm-started from the previous frame), and kept
//! as poses — [`BakedFrame`]s — until the document changes. Playback,
//! scrubbing and (later) every output read the same frames, so they cannot
//! disagree. The solve is sequential (scrubbing backwards would otherwise mean
//! re-solving from 0), so it is time-sliced under a per-tick budget inside the
//! app's frame ([`EngineState::simulation_tick`]) rather than run at once, and
//! the pane shows its progress; the playhead never runs ahead of the bake.
//!
//! Displays are posed by matrix over the geometry the renderer already holds
//! (`SolidDisplay::pose`, as an exploded view's are), re-posing only the
//! occurrences whose pose changed since the shown frame, and
//! [`EngineState::simulation_stop`] puts every display back exactly. Nothing
//! is written to the document. A frame the mates cannot hold keeps the
//! previous frame's poses and is marked failed; it never stops the bake.
//!
//! Interference is checked on every baked frame (`Study::interference`): the
//! occurrences' display meshes, posed by the frame, are screened pair by pair
//! for an edge of one piercing a face of the other (`interference::
//! displays_pierce`: face-to-face contact is not a clash, nor are coincident
//! surfaces). A clashing frame is marked, its parts are tinted while shown,
//! and playback keeps going — there is no stop at contact.
//!
//! The timeline's clips are read from the same study sample: suppress clips
//! go into the bake (a mate off for the frame), and at show time a frame's
//! sample hides the occurrences of its visibility clips, folds an explode
//! clip's step translations (from the exploded view's record) into the
//! occurrences' poses, drives the camera between two PMI views' cameras for
//! a camera clip — the camera as it stood before the first such frame is
//! kept and given back with the displays — sets the displays' opacity for a
//! transparency clip, and cuts the posed displays on a PMI view's section
//! planes (swept along their normals) for a section clip. No PMI view is
//! ever activated by playback: one posing path, one set of originals, one
//! exact restore.

use super::interference::{displays_pierce, overlaps, posed_bbox};
use super::pmi_section::{clip_display_all, side_of_all, Side};
use super::{component_move::now_ms, EngineState};
use crate::camera::Aabb;
use crate::scene::SolidDisplay;
use brep_kernel::{PmiCamera, PmiProjection};
use brep_simulation::model::Study;
use brep_simulation::sample::SectionSample;
use brep_simulation::FrameSample;
use serde_json::{json, Map, Value};
use std::collections::{BTreeMap, HashMap};

/// Solving per app tick: a 121-frame study of a small mechanism bakes in a
/// few ticks while the UI keeps drawing.
const BAKE_BUDGET_MS: f64 = 12.0;

/// One baked frame: every movable occurrence's world pose (row-major 4×4, the
/// kernel's spelling), and whether the mates held it.
#[derive(Debug, Clone, PartialEq)]
pub struct BakedFrame {
    pub poses: BTreeMap<String, [f64; 16]>,
    /// The mates could not hold this frame; the previous frame's poses stand in.
    pub failed: bool,
    /// The occurrence pairs that interfere at this frame (each `a < b`), by
    /// the participant ids of the interference check; empty when the study
    /// does not check or nothing clashes.
    pub clashes: Vec<(String, String)>,
}

/// The row-major identity pose.
pub(super) const IDENTITY: [f64; 16] = [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0];

/// The tint of a clashing part's displays while its frame is shown.
const CLASH_TINT: [f32; 3] = [0.92, 0.25, 0.2];

/// The study being played or scrubbed, its bake so far and what the displays
/// show of it.
#[derive(Debug)]
pub struct SimulationPlayback {
    pub study: String,
    /// The study as baked, and the document revision it was baked against;
    /// either moving starts the bake over.
    baked_study: Study,
    revision: u64,
    /// The mates' document values, what a speed motor starts from.
    base: HashMap<String, f64>,
    pub frames: Vec<BakedFrame>,
    total: usize,
    /// The previous frame's reply poses: the next frame's warm start.
    reference: Map<String, Value>,
    /// The playhead, seconds.
    pub time: f64,
    pub playing: bool,
    pub looping: bool,
    /// The frame the displays show.
    shown: Option<usize>,
    /// The frame asked for (by a seek or the playhead); shown once it is baked.
    wanted: Option<usize>,
    /// Displays as they were before their first pose, by solid name.
    originals: HashMap<String, SolidDisplay>,
    /// Occurrence → the world pose its displays show.
    applied: HashMap<String, [f64; 16]>,
    /// The last frame the mates could not hold, and why.
    pub error: Option<String>,
    /// The interference check's participants: occurrence → its member solids
    /// (`interference_participants`, taken at prepare).
    participants: Vec<(String, Vec<String>)>,
    /// The solids tinted for the shown frame's clashes.
    tinted: Vec<String>,
    /// The solids a visibility clip hides at the shown frame.
    hidden: Vec<String>,
    /// The solids a transparency clip made see-through at the shown frame.
    translucent: Vec<String>,
    /// The solids a section clip cut or hid at the shown frame; put back
    /// (and re-posed) before the next frame is posed.
    sectioned: Vec<String>,
    /// The camera as it stood before a camera clip first drove it; given
    /// back with the displays.
    camera_before: Option<PmiCamera>,
    /// The shown frame's clip sample (what automation reads).
    shown_sample: Option<FrameSample>,
}

impl SimulationPlayback {
    /// The study as baked.
    pub(crate) fn baked_study(&self) -> &Study {
        &self.baked_study
    }

    /// The document revision the bake was made against.
    pub(crate) fn revision(&self) -> u64 {
        self.revision
    }

    /// The mates' document values (a speed motor's start).
    pub(crate) fn base(&self) -> &HashMap<String, f64> {
        &self.base
    }

    /// The display as the document poses it, when playback has recorded it.
    pub(crate) fn original(&self, name: &str) -> Option<&SolidDisplay> {
        self.originals.get(name)
    }
}

impl EngineState {
    /// The playback, if a study has been played or scrubbed.
    pub(crate) fn simulation_playback(&self) -> Option<&SimulationPlayback> {
        self.simulation_play.as_ref()
    }

    /// Bake `study` (the active one when `None`) to the end, now: the whole
    /// bake in one call, as an export or a published motion needs it. Pauses
    /// playback; the playhead is kept. Returns the study id.
    pub fn simulation_bake_all(&mut self, study: Option<&str>) -> Result<String, String> {
        let id = self.simulation_prepare(study)?;
        if let Some(play) = self.simulation_play.as_mut() {
            play.playing = false;
        }
        self.ensure_assembly_session();
        let mut guard = 0usize;
        while self.simulation_play.as_ref().is_some_and(|play| play.frames.len() < play.total) {
            guard += 1;
            if guard > 1_000_000 {
                return Err("the bake did not finish".into());
            }
            self.simulation_bake_next();
        }
        Ok(id)
    }

    /// A publish wrote `motion` into `study`'s block entry: the bake is of the
    /// same study still, so it is kept — its record takes the motion and the
    /// revision the write made, which `simulation_prepare` compares.
    pub(super) fn simulation_keep_bake_through_publish(&mut self, study: &str, motion: Value) {
        let revision = self.history.revision();
        if let Some(play) = self.simulation_play.as_mut().filter(|play| play.study == study) {
            play.baked_study.motion = Some(motion);
            play.revision = revision;
        }
    }

    /// Play `study` (the active one when `None`) from where its playhead
    /// stands — from 0 when it stands at the end — baking it first if need
    /// be; `looping` sets whether it repeats. Returns the study id.
    pub fn simulation_play(&mut self, study: Option<&str>, looping: Option<bool>) -> Result<String, String> {
        let id = self.simulation_prepare(study)?;
        let play = self.simulation_play.as_mut().expect("prepared");
        if let Some(looping) = looping {
            play.looping = looping;
        }
        if play.time >= play.baked_study.duration - 1e-9 {
            play.time = 0.0;
        }
        play.playing = true;
        play.wanted = Some(play.baked_study.frame_at(play.time));
        Ok(id)
    }

    /// Hold the playhead where it is; the frame stays shown.
    pub fn simulation_pause(&mut self) {
        if let Some(play) = self.simulation_play.as_mut() {
            play.playing = false;
        }
    }

    /// Every display back at its document pose, the playhead at 0. The bake
    /// is kept, so Play starts again at once.
    pub fn simulation_stop(&mut self) {
        self.simulation_restore_displays();
        if let Some(play) = self.simulation_play.as_mut() {
            play.playing = false;
            play.time = 0.0;
            play.wanted = None;
        }
    }

    /// Put the playhead at `time` (seconds, clamped to the study) and show
    /// that frame, pausing; a frame not baked yet is shown when it is.
    pub fn simulation_seek(&mut self, study: Option<&str>, time: f64) -> Result<(), String> {
        if !time.is_finite() {
            return Err("seek: the time is not a number".into());
        }
        self.simulation_prepare(study)?;
        let play = self.simulation_play.as_mut().expect("prepared");
        play.playing = false;
        play.time = time.clamp(0.0, play.baked_study.duration);
        play.wanted = Some(play.baked_study.frame_at(play.time));
        self.simulation_show_wanted();
        Ok(())
    }

    /// Whether playback repeats at the study's end.
    pub fn simulation_set_looping(&mut self, study: Option<&str>, looping: bool) -> Result<(), String> {
        self.simulation_prepare(study)?;
        self.simulation_play.as_mut().expect("prepared").looping = looping;
        Ok(())
    }

    /// Whether a study is being played or scrubbed (its displays may be posed).
    pub fn simulation_playing_or_shown(&self) -> bool {
        self.simulation_play.as_ref().is_some_and(|play| play.playing || play.shown.is_some())
    }

    /// Make `study` (the active one when `None`) the playback, keeping an
    /// up-to-date bake of the same study and starting over otherwise.
    fn simulation_prepare(&mut self, study: Option<&str>) -> Result<String, String> {
        let block = self.simulation()?.ok_or("no study yet: New study starts one")?;
        let study = match study {
            Some(id) => block.study(id).ok_or_else(|| format!("no study '{id}'"))?,
            None => block.active_study().ok_or("no study yet: New study starts one")?,
        }
        .clone();
        let revision = self.history.revision();
        let current = self
            .simulation_play
            .as_ref()
            .is_some_and(|play| play.study == study.id && play.revision == revision && play.baked_study == study);
        if current {
            return Ok(study.id);
        }
        // Another study's (or a stale bake's) poses go, and an exploded view
        // is put away, so every display sits at its document pose: what the
        // frames' matrices are relative to.
        self.simulation_restore_displays();
        if self.pmi_active_view.is_some() {
            self.pmi_deactivate_view();
        }
        let previous = self.simulation_play.take();
        let (time, looping) = previous
            .filter(|play| play.study == study.id)
            .map_or((0.0, false), |play| (play.time.min(study.duration), play.looping));
        let base = self.simulation_motor_mates().into_iter().map(|mate| (mate.id, mate.value)).collect();
        let participants = if study.interference.enabled { self.interference_participants() } else { Vec::new() };
        let total = study.frame_count();
        let id = study.id.clone();
        self.simulation_play = Some(SimulationPlayback {
            study: id.clone(),
            baked_study: study,
            revision,
            base,
            frames: Vec::with_capacity(total),
            total,
            reference: Map::new(),
            time,
            playing: false,
            looping,
            shown: None,
            wanted: None,
            originals: HashMap::new(),
            applied: HashMap::new(),
            error: None,
            participants,
            tinted: Vec::new(),
            hidden: Vec::new(),
            translucent: Vec::new(),
            sectioned: Vec::new(),
            camera_before: None,
            shown_sample: None,
        });
        Ok(id)
    }

    /// Advance by `dt` seconds: bake frames under the budget, move the
    /// playhead and show its frame. `true` while another frame is needed
    /// (playing, or still baking). The app calls it once per frame.
    pub fn simulation_tick(&mut self, dt: f64) -> bool {
        let Some(play) = self.simulation_play.as_ref() else { return false };
        // The document moved under the bake: start over, keeping the playhead.
        if play.revision != self.history.revision() {
            let (study, time, playing, looping) = (play.study.clone(), play.time, play.playing, play.looping);
            self.simulation_restore_displays();
            self.simulation_play = None;
            if self.simulation_prepare(Some(&study)).is_err() {
                return false;
            }
            let play = self.simulation_play.as_mut().expect("prepared");
            play.time = time.min(play.baked_study.duration);
            play.playing = playing;
            play.looping = looping;
            play.wanted = Some(play.baked_study.frame_at(play.time));
        }
        let start = now_ms();
        if self.simulation_play.as_ref().is_some_and(|play| play.frames.len() < play.total) {
            // The frame solve runs against the MAIN-side kernel session, which
            // the history runner's thread owns after a run: bring it over (a
            // no-op once it is current), as a live drag does before its solve.
            self.ensure_assembly_session();
        }
        while self.simulation_play.as_ref().is_some_and(|play| play.frames.len() < play.total) && now_ms() - start < BAKE_BUDGET_MS {
            self.simulation_bake_next();
        }
        let play = self.simulation_play.as_mut().expect("checked above");
        if play.playing {
            let duration = play.baked_study.duration;
            play.time += dt.max(0.0);
            if play.time >= duration - 1e-9 {
                if play.looping {
                    play.time = (play.time % duration).max(0.0);
                } else {
                    play.time = duration;
                    play.playing = false;
                }
            }
            // The playhead waits for the bake rather than running ahead of it.
            let baked = play.frames.len();
            if play.baked_study.frame_at(play.time) >= baked {
                play.time = play.baked_study.frame_time(baked.saturating_sub(1));
                if baked == 0 {
                    play.time = 0.0;
                }
            }
            play.wanted = Some(play.baked_study.frame_at(play.time));
        }
        self.simulation_show_wanted();
        let play = self.simulation_play.as_ref().expect("checked above");
        play.playing || play.frames.len() < play.total
    }

    /// Solve the next unbaked frame against the session (one step of the
    /// tick's bake loop; the tests step it by hand).
    pub(super) fn simulation_bake_next(&mut self) {
        let Some(play) = self.simulation_play.as_mut() else { return };
        let index = play.frames.len();
        if index >= play.total {
            return;
        }
        let time = play.baked_study.frame_time(index);
        let base = &play.base;
        let sample = play.baked_study.sample(time, &|id| base.get(id).copied());
        let mut overrides = Map::new();
        for (id, mate) in &sample.mates {
            let mut change = Map::new();
            if let Some(value) = mate.value {
                change.insert("value".into(), json!(value));
            }
            if let Some(enabled) = mate.enabled {
                change.insert("enabled".into(), json!(enabled));
            }
            overrides.insert(id.clone(), Value::Object(change));
        }
        let request = json!({ "overrides": overrides, "reference": play.reference });
        let (poses, failed): (BTreeMap<String, [f64; 16]>, bool) = match brep_kernel::assembly_frame_solve_impl(&request.to_string()) {
            Ok(reply) => {
                let reply: Value = serde_json::from_str(&reply).unwrap_or(Value::Null);
                let poses_json = reply.get("poses").and_then(Value::as_object).cloned().unwrap_or_default();
                let poses = poses_json.iter().map(|(id, pose)| (id.clone(), crate::pipeline::pose_params_to_transform(pose))).collect();
                play.reference = poses_json;
                (poses, false)
            }
            Err(error) => {
                let poses = play.frames.last().map(|frame| frame.poses.clone()).unwrap_or_default();
                play.error = Some(format!("frame {index} ({time:.3} s): {error}"));
                (poses, true)
            }
        };
        // The interference screen reads the scene, so the playback borrow ends
        // here; its participants are lent out and put back.
        let participants = std::mem::take(&mut play.participants);
        let previous = play.frames.last().cloned();
        let clashes = if participants.is_empty() { Vec::new() } else { self.simulation_frame_clashes(&participants, &poses, previous.as_ref()) };
        let Some(play) = self.simulation_play.as_mut() else { return };
        play.participants = participants;
        play.frames.push(BakedFrame { poses, failed, clashes });
    }

    /// The occurrence pairs whose posed displays interfere at a frame: each
    /// occurrence's displays are carried by its pose over the document
    /// (identity for one the frame did not move), the pairs whose posed boxes
    /// overlap are screened mesh against mesh, and a pair neither of whose
    /// occurrences moved since the previous frame keeps that frame's verdict.
    /// The geometry read is the display AS THE DOCUMENT POSES IT: a solid
    /// already posed for a shown frame (the bake may still be running) is
    /// read from its recorded original, never from the scene.
    fn simulation_frame_clashes(&self, participants: &[(String, Vec<String>)], poses: &BTreeMap<String, [f64; 16]>, previous: Option<&BakedFrame>) -> Vec<(String, String)> {
        let originals = self.simulation_play.as_ref().map(|play| &play.originals);
        let display_of = |name: &String| -> Option<&SolidDisplay> { originals.and_then(|originals| originals.get(name)).or_else(|| self.scene.solid(name)) };
        let delta_of = |occ: &str| -> [f64; 16] {
            let top = occ.split(':').next().unwrap_or(occ);
            let (key, world) = match poses.get(occ) {
                Some(world) => (occ, world),
                None => match poses.get(top) {
                    Some(world) => (top, world),
                    None => return IDENTITY,
                },
            };
            match self.occurrence_document_matrix(key) {
                Some(document) => mat_mul(world, &rigid_inverse(&document)),
                None => IDENTITY,
            }
        };
        let mut posed: Vec<(&str, [f64; 16], Aabb, &[String])> = Vec::new();
        for (occ, solids) in participants {
            let delta = delta_of(occ);
            let mut bbox = Aabb::empty();
            for name in solids {
                if let Some(solid) = display_of(name) {
                    bbox.union(&posed_bbox(&solid.bbox, &delta));
                }
            }
            if !bbox.is_empty() {
                posed.push((occ.as_str(), delta, bbox, solids.as_slice()));
            }
        }
        let same_as_previous = |occ: &str, prev: &BakedFrame| {
            let top = occ.split(':').next().unwrap_or(occ);
            poses.get(occ) == prev.poses.get(occ) && poses.get(top) == prev.poses.get(top)
        };
        let mut clashes = Vec::new();
        for i in 0..posed.len() {
            for j in i + 1..posed.len() {
                if !overlaps(&posed[i].2, &posed[j].2) {
                    continue;
                }
                let pair = (posed[i].0.to_string(), posed[j].0.to_string());
                if let Some(prev) = previous {
                    if same_as_previous(&pair.0, prev) && same_as_previous(&pair.1, prev) {
                        if prev.clashes.contains(&pair) {
                            clashes.push(pair);
                        }
                        continue;
                    }
                }
                let hit = posed[i].3.iter().filter_map(display_of).any(|a| posed[j].3.iter().filter_map(display_of).any(|b| displays_pierce(a, &posed[i].1, b, &posed[j].1)));
                if hit {
                    clashes.push(pair);
                }
            }
        }
        clashes
    }

    /// Show the wanted frame if it is baked and not already shown.
    fn simulation_show_wanted(&mut self) {
        let Some(play) = self.simulation_play.as_ref() else { return };
        let Some(wanted) = play.wanted else { return };
        if wanted >= play.frames.len() || play.shown == Some(wanted) {
            return;
        }
        self.simulation_pose_frame(wanted);
    }

    /// Where every moved occurrence sits at frame `index` (row-major world
    /// matrices): the solve's poses, plus an explode clip's steps — its view's
    /// enabled steps, each `fraction` of the way at the frame's progress (the
    /// steps below the progress whole, the one at it partly), translating
    /// their target occurrences on top of the solve. `None` before the frame
    /// is baked.
    pub(crate) fn simulation_frame_worlds(&self, index: usize) -> Option<BTreeMap<String, [f64; 16]>> {
        let play = self.simulation_play.as_ref()?;
        let frame = play.frames.get(index)?;
        let mut worlds = frame.poses.clone();
        let time = play.baked_study.frame_time(index);
        let base = &play.base;
        let sample = play.baked_study.sample(time, &|id| base.get(id).copied());
        for explode in &sample.explode {
            let Some(view) = self.exploded_views().into_iter().find(|view| view.id == explode.view) else { continue };
            let steps: Vec<_> = view.steps.iter().filter(|step| step.enabled).collect();
            let count = steps.len() as f64;
            for (k, step) in steps.iter().enumerate() {
                let fraction = (explode.progress * count - k as f64).clamp(0.0, 1.0);
                if fraction <= 0.0 {
                    continue;
                }
                for target in &step.targets {
                    let Some(world) = worlds.get(target).copied().or_else(|| self.occurrence_document_matrix(target)) else { continue };
                    let mut shift = IDENTITY;
                    for axis in 0..3 {
                        shift[axis * 4 + 3] = step.translate[axis] * fraction;
                    }
                    worlds.insert(target.clone(), mat_mul(&shift, &world));
                }
            }
        }
        Some(worlds)
    }

    /// Pose the displays at frame `index`, touching only the occurrences
    /// whose pose differs from what they show; then the frame's clips (hidden
    /// occurrences, the camera) and the clash tint.
    fn simulation_pose_frame(&mut self, index: usize) {
        let Some(play) = self.simulation_play.as_ref() else { return };
        if index >= play.frames.len() {
            return;
        }
        let time = play.baked_study.frame_time(index);
        let base = play.base.clone();
        let sample = play.baked_study.sample(time, &|id| base.get(id).copied());
        let Some(worlds) = self.simulation_frame_worlds(index) else { return };
        let Some(play) = self.simulation_play.as_ref() else { return };
        let frame = worlds;
        // Occurrences posed last time and absent from this frame go back.
        let gone: Vec<String> = play.applied.keys().filter(|id| !frame.contains_key(*id)).cloned().collect();
        self.simulation_unsection();
        let Some(play) = self.simulation_play.as_ref() else { return };
        let mut changed = false;
        for id in gone {
            self.simulation_pose_occurrence(&id, None);
            changed = true;
        }
        for (id, world) in &frame {
            let Some(document) = self.occurrence_document_matrix(id) else { continue };
            let shown = self.simulation_play.as_ref().and_then(|play| play.applied.get(id).copied()).unwrap_or(document);
            if same_matrix(&shown, world) {
                continue;
            }
            if same_matrix(world, &document) {
                // Back at the document pose: the originals, not an identity pose.
                self.simulation_pose_occurrence(id, None);
            } else {
                let delta = mat_mul(world, &rigid_inverse(&document));
                self.simulation_pose_occurrence(id, Some((delta, *world)));
            }
            changed = true;
        }
        if let Some(play) = self.simulation_play.as_mut() {
            play.shown = Some(index);
        }
        if changed {
            self.dirty = true;
        }
        self.simulation_apply_section(sample.section.as_ref());
        self.simulation_tint_clashes(index);
        self.simulation_apply_hidden(&sample.hidden);
        self.simulation_apply_opacity(&sample.opacity);
        if let Some(camera) = &sample.camera {
            self.simulation_apply_camera(&camera.from, &camera.to, camera.t);
        }
        if let Some(play) = self.simulation_play.as_mut() {
            play.shown_sample = Some(sample);
        }
    }

    /// The solids the last frame's section cut or hid go back to their
    /// originals and, where their occurrence is posed, are posed again, so
    /// this frame's section (or none) starts from whole geometry.
    fn simulation_unsection(&mut self) {
        let Some(play) = self.simulation_play.as_mut() else { return };
        let sectioned = std::mem::take(&mut play.sectioned);
        if sectioned.is_empty() {
            return;
        }
        let applied: Vec<(String, [f64; 16])> = play.applied.iter().map(|(id, world)| (id.clone(), *world)).collect();
        let mut reposed: Vec<String> = Vec::new();
        for name in &sectioned {
            if let (Some(original), Some(display)) = (play.originals.get(name), self.scene.solid_mut(name)) {
                *display = original.clone();
            }
            // The longest posed occurrence this solid belongs to.
            if let Some((id, _)) = applied.iter().filter(|(id, _)| name.starts_with(&format!("{id}:"))).max_by_key(|(id, _)| id.len()) {
                if !reposed.contains(id) {
                    reposed.push(id.clone());
                }
            }
        }
        for id in reposed {
            let Some(world) = applied.iter().find(|(key, _)| *key == id).map(|(_, world)| *world) else { continue };
            let Some(document) = self.occurrence_document_matrix(&id) else { continue };
            self.simulation_pose_occurrence(&id, Some((mat_mul(&world, &rigid_inverse(&document)), world)));
        }
        self.dirty = true;
    }

    /// Cut the displays on the PMI view's section planes, moved `offset`
    /// along their normals: a solid wholly on the removed side is hidden, one
    /// the planes cross is clipped and capped, one the view's section
    /// excludes stays whole. Nothing with no section.
    fn simulation_apply_section(&mut self, section: Option<&SectionSample>) {
        let Some(section) = section else { return };
        let state = self.pmi_state_shared();
        let Some(view) = state.find_view(&section.view) else { return };
        let report = self.pmi_report.as_ref().and_then(|report| report.view(&section.view));
        let planes: Vec<([f64; 3], [f64; 3])> = view
            .section_planes_in(report)
            .into_iter()
            .map(|(point, normal)| (std::array::from_fn(|i| point[i] + normal[i] * section.offset), normal))
            .collect();
        if planes.is_empty() {
            return;
        }
        let exclude = view.display.section.clone();
        let names: Vec<(String, Side)> = self
            .scene
            .solids()
            .iter()
            .filter(|solid| solid.visible)
            .map(|solid| (solid.name.clone(), if exclude.as_ref().is_some_and(|cut| cut.excludes(&solid.name)) { Side::Kept } else { side_of_all(solid, &planes) }))
            .collect();
        let mut sectioned = Vec::new();
        for (name, side) in names {
            if side == Side::Kept {
                continue;
            }
            let Some(display) = self.scene.solid(&name).cloned() else { continue };
            if let Some(play) = self.simulation_play.as_mut() {
                play.originals.entry(name.clone()).or_insert(display);
            }
            if let Some(display) = self.scene.solid_mut(&name) {
                match side {
                    Side::Removed => display.visible = false,
                    Side::Cut => clip_display_all(display, &planes),
                    Side::Kept => {}
                }
                display.revision = crate::scene::next_revision();
            }
            sectioned.push(name);
        }
        if let Some(play) = self.simulation_play.as_mut() {
            play.sectioned = sectioned;
        }
        self.dirty = true;
    }

    /// Set the opacity of the solids a transparency clip names at this frame
    /// and give the last frame's see-through solids their original back.
    fn simulation_apply_opacity(&mut self, opacity: &BTreeMap<String, f64>) {
        let mut want: Vec<(String, f32)> = Vec::new();
        for (id, value) in opacity {
            for name in self.simulation_occurrence_solids(id) {
                want.push((name, value.clamp(0.0, 1.0) as f32));
            }
        }
        want.sort_by(|a, b| a.0.cmp(&b.0));
        want.dedup_by(|a, b| a.0 == b.0);
        let Some(play) = self.simulation_play.as_ref() else { return };
        let previous = play.translucent.clone();
        let mut changed = false;
        for name in previous.iter().filter(|name| !want.iter().any(|(n, _)| n == *name)) {
            let opacity = self.simulation_play.as_ref().and_then(|play| play.originals.get(name)).map(|original| original.opacity);
            if let (Some(opacity), Some(display)) = (opacity, self.scene.solid_mut(name)) {
                if display.opacity != opacity {
                    display.opacity = opacity;
                    display.revision = crate::scene::next_revision();
                    changed = true;
                }
            }
        }
        for (name, value) in &want {
            let Some(display) = self.scene.solid(name).cloned() else { continue };
            if let Some(play) = self.simulation_play.as_mut() {
                play.originals.entry(name.clone()).or_insert(display);
            }
            if let Some(display) = self.scene.solid_mut(name) {
                if display.opacity != Some(*value) {
                    display.opacity = Some(*value);
                    display.revision = crate::scene::next_revision();
                    changed = true;
                }
            }
        }
        if let Some(play) = self.simulation_play.as_mut() {
            play.translucent = want.into_iter().map(|(name, _)| name).collect();
        }
        if changed {
            self.dirty = true;
        }
    }

    /// The solids of occurrence `id`: a nested occurrence's by name prefix, a
    /// top component's members.
    fn simulation_occurrence_solids(&self, id: &str) -> Vec<String> {
        if id.contains(':') {
            let prefix = format!("{id}:");
            self.scene.solids().iter().filter(|solid| solid.name.starts_with(&prefix)).map(|solid| solid.name.clone()).collect()
        } else {
            self.member_solid_names(id)
        }
    }

    /// Hide the solids of the frame's visibility clips and show again the
    /// ones hidden for the last frame that no clip hides now (from their
    /// originals, so a solid the document hides stays hidden).
    fn simulation_apply_hidden(&mut self, occurrences: &[String]) {
        let mut want: Vec<String> = occurrences.iter().flat_map(|id| self.simulation_occurrence_solids(id)).collect();
        want.sort();
        want.dedup();
        let Some(play) = self.simulation_play.as_ref() else { return };
        let previous = play.hidden.clone();
        let mut changed = false;
        for name in previous.iter().filter(|name| !want.contains(name)) {
            let visible = self.simulation_play.as_ref().and_then(|play| play.originals.get(name)).map(|original| original.visible);
            if let (Some(visible), Some(display)) = (visible, self.scene.solid_mut(name)) {
                if display.visible != visible {
                    display.visible = visible;
                    changed = true;
                }
            }
        }
        for name in &want {
            let Some(display) = self.scene.solid(name).cloned() else { continue };
            if let Some(play) = self.simulation_play.as_mut() {
                play.originals.entry(name.clone()).or_insert(display);
            }
            if let Some(display) = self.scene.solid_mut(name) {
                if display.visible {
                    display.visible = false;
                    changed = true;
                }
            }
        }
        if let Some(play) = self.simulation_play.as_mut() {
            play.hidden = want;
        }
        if changed {
            self.dirty = true;
        }
    }

    /// Put the camera `t` of the way from PMI view `from`'s camera to view
    /// `to`'s (eye, target, up and the projection's scale blended; the
    /// projection kind switches half way), keeping the camera as it stood
    /// before the first such frame.
    fn simulation_apply_camera(&mut self, from: &str, to: &str, t: f64) {
        let Some(camera) = self.simulation_camera_between(from, to, t) else { return };
        let standing = self.snapshot_camera();
        if let Some(play) = self.simulation_play.as_mut() {
            if play.camera_before.is_none() {
                play.camera_before = Some(standing);
            }
        }
        self.simulation_apply_pmi_camera(&camera);
    }

    /// The camera `t` of the way from PMI view `from`'s camera to view `to`'s
    /// (eye, target, up and the projection's scale blended; the projection
    /// kind switches half way); `None` when either view has no camera.
    pub(crate) fn simulation_camera_between(&self, from: &str, to: &str, t: f64) -> Option<PmiCamera> {
        let state = self.pmi_state_shared();
        let a = state.find_view(from).and_then(|v| v.camera.clone())?;
        let b = state.find_view(to).and_then(|v| v.camera.clone())?;
        let t = t.clamp(0.0, 1.0);
        let lerp = |p: [f64; 3], q: [f64; 3]| -> [f64; 3] { std::array::from_fn(|i| p[i] + (q[i] - p[i]) * t) };
        let scale_of = |camera: &PmiCamera| match camera.projection {
            PmiProjection::Orthographic { half_height } => half_height,
            PmiProjection::Perspective { fov_y_deg } => fov_y_deg,
        };
        let orthographic = |camera: &PmiCamera| matches!(camera.projection, PmiProjection::Orthographic { .. });
        let same_kind = orthographic(&a) == orthographic(&b);
        let (ortho, scale) = if same_kind {
            (orthographic(&a), scale_of(&a) + (scale_of(&b) - scale_of(&a)) * t)
        } else if t < 0.5 {
            (orthographic(&a), scale_of(&a))
        } else {
            (orthographic(&b), scale_of(&b))
        };
        Some(PmiCamera {
            eye: lerp(a.eye, b.eye),
            target: lerp(a.target, b.target),
            up: lerp(a.up, b.up),
            projection: if ortho { PmiProjection::Orthographic { half_height: scale } } else { PmiProjection::Perspective { fov_y_deg: scale } },
            viewport: a.viewport,
        })
    }

    /// Apply a PMI camera record as the live camera.
    fn simulation_apply_pmi_camera(&mut self, camera: &PmiCamera) {
        let (kind, scale) = match camera.projection {
            PmiProjection::Orthographic { half_height } => ("orthographic", half_height),
            PmiProjection::Perspective { fov_y_deg } => ("perspective", fov_y_deg),
        };
        let json = json!({ "kind": kind, "eye": camera.eye, "target": camera.target, "up": camera.up, "scale": scale }).to_string();
        if self.apply_camera_state_json(&json).is_ok() {
            self.controls_sync_after_camera_apply();
        }
    }

    /// Tint the displays of the occurrences that clash at frame `index`, and
    /// give back their colour to the ones tinted for the last frame that no
    /// longer clash. Runs after the frame's posing (a re-posed display is
    /// rebuilt from its original, which holds the colour the document gave it).
    fn simulation_tint_clashes(&mut self, index: usize) {
        let Some(play) = self.simulation_play.as_ref() else { return };
        let Some(frame) = play.frames.get(index) else { return };
        let mut want: Vec<String> = Vec::new();
        for (a, b) in &frame.clashes {
            for occ in [a, b] {
                if let Some((_, solids)) = play.participants.iter().find(|(id, _)| id == occ) {
                    want.extend(solids.iter().cloned());
                }
            }
        }
        want.sort();
        want.dedup();
        let previous = play.tinted.clone();
        let mut changed = false;
        for name in previous.iter().filter(|name| !want.contains(name)) {
            let color = self.simulation_play.as_ref().and_then(|play| play.originals.get(name)).map(|original| original.color_override);
            if let (Some(color), Some(display)) = (color, self.scene.solid_mut(name)) {
                if display.color_override != color {
                    display.color_override = color;
                    display.revision = crate::scene::next_revision();
                    changed = true;
                }
            }
        }
        for name in &want {
            // The original is kept before the tint: an unmoved occurrence may
            // not have been posed, so nothing recorded it yet.
            let Some(display) = self.scene.solid(name).cloned() else { continue };
            if let Some(play) = self.simulation_play.as_mut() {
                play.originals.entry(name.clone()).or_insert(display);
            }
            if let Some(display) = self.scene.solid_mut(name) {
                if display.color_override != Some(CLASH_TINT) {
                    display.color_override = Some(CLASH_TINT);
                    // The renderer re-derives a style on a new revision only.
                    display.revision = crate::scene::next_revision();
                    changed = true;
                }
            }
        }
        if let Some(play) = self.simulation_play.as_mut() {
            play.tinted = want;
        }
        if changed {
            self.dirty = true;
        }
    }

    /// Pose occurrence `id`'s displays by `delta` over their originals (and
    /// record the `world` pose they then show), or put them back (`None`).
    /// A top component's pass re-poses the solids of its nested occurrences
    /// too, so those are forgotten and posed again after it (they follow it
    /// in a frame's map).
    fn simulation_pose_occurrence(&mut self, id: &str, pose: Option<([f64; 16], [f64; 16])>) {
        let names: Vec<String> = if id.contains(':') {
            let prefix = format!("{id}:");
            self.scene.solids().iter().filter(|solid| solid.name.starts_with(&prefix)).map(|solid| solid.name.clone()).collect()
        } else {
            self.member_solid_names(id)
        };
        let Some(play) = self.simulation_play.as_mut() else { return };
        for name in names {
            let Some(display) = self.scene.solid(&name) else { continue };
            let original = play.originals.entry(name.clone()).or_insert_with(|| display.clone()).clone();
            if let Some(display) = self.scene.solid_mut(&name) {
                *display = original;
                if let Some((delta, _)) = &pose {
                    pose_display_by_matrix(display, delta);
                }
            }
        }
        let nested = format!("{id}:");
        play.applied.retain(|key, _| !key.starts_with(&nested));
        match pose {
            Some((_, world)) => {
                play.applied.insert(id.to_string(), world);
            }
            None => {
                play.applied.remove(id);
            }
        }
    }

    /// Where occurrence `id`'s displays sit in the document: a top
    /// component's solved pose, or that composed with a nested occurrence's
    /// solved pose inside it (row-major 4×4).
    pub(super) fn occurrence_document_matrix(&self, id: &str) -> Option<[f64; 16]> {
        let (top, rest) = match id.split_once(':') {
            Some((top, rest)) => (top, Some(rest)),
            None => (id, None),
        };
        let record = self.assembly_components().iter().find(|record| record.id == top)?;
        match rest {
            None => Some(record.transform),
            Some(rest) => Some(mat_mul(&record.transform, record.nested_poses.get(rest)?)),
        }
    }

    /// Every posed display back exactly; the bake and the playhead are kept,
    /// and the shown frame is asked for again (a run that rebuilds the
    /// displays re-poses it; a stop forgets it).
    pub(crate) fn simulation_restore_displays(&mut self) {
        let Some(play) = self.simulation_play.as_mut() else { return };
        play.applied.clear();
        play.tinted.clear();
        play.hidden.clear();
        play.translucent.clear();
        play.sectioned.clear();
        play.shown_sample = None;
        let camera_before = play.camera_before.take();
        let originals = std::mem::take(&mut play.originals);
        if let Some(camera) = camera_before {
            self.simulation_apply_pmi_camera(&camera);
        }
        let Some(play) = self.simulation_play.as_mut() else { return };
        if let Some(shown) = play.shown.take() {
            play.wanted.get_or_insert(shown);
        }
        if originals.is_empty() {
            return;
        }
        for (name, original) in originals {
            if let Some(display) = self.scene.solid_mut(&name) {
                *display = original;
            }
        }
        self.dirty = true;
    }

    /// Post-apply tail: the displays are fresh (un-posed before the
    /// reconcile), so the frame that was shown goes back on them when the
    /// bake is still current; a moved document re-bakes on the next tick.
    pub(crate) fn simulation_after_apply(&mut self) {
        let Some(play) = self.simulation_play.as_ref() else { return };
        if play.revision == self.history.revision() {
            self.simulation_show_wanted();
        }
    }

    /// Forget the playback entirely (displays restored): the document closed
    /// or the workbench left.
    pub fn simulation_forget_playback(&mut self) {
        self.simulation_restore_displays();
        self.simulation_play = None;
    }

    /// What automation and the pane read (`__brepSimulation.playback`):
    /// `{study, time, playheadFrame, shown, baked, total, playing, looping,
    /// failed: [frame…], error, poses: {occurrence: {translate,
    /// rotateEulerDeg}}, interference, clashes: [frame…], shownClashes:
    /// [[a, b]…], hidden: [occurrence…], explode: [{view, progress}…],
    /// camera: {from, to, t} | null, cameraHeld, opacity: {occurrence:
    /// value}, section: {view, offset} | null}` — `poses` are the
    /// occurrences the shown frame moved from their document poses (explode
    /// clips included), `clashes` the baked frames where parts interfere,
    /// `shownClashes` the pairs at the shown frame, and `hidden`, `explode`
    /// and `camera` the shown frame's clips (`cameraHeld`: a camera clip has
    /// taken the camera, which Stop gives back); `null` with no playback.
    pub fn simulation_playback_json(&self) -> Value {
        let Some(play) = self.simulation_play.as_ref() else { return Value::Null };
        let failed: Vec<usize> = play.frames.iter().enumerate().filter(|(_, frame)| frame.failed).map(|(index, _)| index).collect();
        let poses: BTreeMap<&str, Value> = play
            .applied
            .iter()
            .map(|(id, world)| (id.as_str(), brep_kernel::transform_to_pose_params(&brep_kernel::AffineTransform { elements: *world })))
            .collect();
        json!({
            "study": play.study,
            "time": play.time,
            "playheadFrame": play.baked_study.frame_at(play.time),
            "shown": play.shown,
            "baked": play.frames.len(),
            "total": play.total,
            "playing": play.playing,
            "looping": play.looping,
            "failed": failed,
            "error": play.error,
            "poses": poses,
            "interference": play.baked_study.interference.enabled,
            "clashes": play.frames.iter().enumerate().filter(|(_, frame)| !frame.clashes.is_empty()).map(|(index, _)| index).collect::<Vec<usize>>(),
            "shownClashes": play.shown.and_then(|index| play.frames.get(index)).map(|frame| frame.clashes.clone()).unwrap_or_default(),
            "hidden": play.shown_sample.as_ref().map(|sample| sample.hidden.clone()).unwrap_or_default(),
            "explode": play.shown_sample.as_ref().map(|sample| sample.explode.iter().map(|e| json!({ "view": e.view, "progress": e.progress })).collect::<Vec<Value>>()).unwrap_or_default(),
            "camera": play.shown_sample.as_ref().and_then(|sample| sample.camera.as_ref()).map(|c| json!({ "from": c.from, "to": c.to, "t": c.t })),
            "cameraHeld": play.camera_before.is_some(),
            "opacity": play.shown_sample.as_ref().map(|sample| sample.opacity.clone()).unwrap_or_default(),
            "section": play.shown_sample.as_ref().and_then(|sample| sample.section.as_ref()).map(|s| json!({ "view": s.view, "offset": s.offset })),
        })
    }
}

/// `a · b`, row-major 4×4.
pub(super) fn mat_mul(a: &[f64; 16], b: &[f64; 16]) -> [f64; 16] {
    std::array::from_fn(|k| {
        let (row, col) = (k / 4, k % 4);
        (0..4).map(|n| a[row * 4 + n] * b[n * 4 + col]).sum()
    })
}

/// The inverse of a rigid row-major 4×4: `[Rᵀ | −Rᵀ·t]`.
pub(super) fn rigid_inverse(m: &[f64; 16]) -> [f64; 16] {
    let mut out = [0.0; 16];
    for row in 0..3 {
        for col in 0..3 {
            out[row * 4 + col] = m[col * 4 + row];
        }
        out[row * 4 + 3] = -(0..3).map(|k| m[k * 4 + row] * m[k * 4 + 3]).sum::<f64>();
    }
    out[15] = 1.0;
    out
}

/// Two poses within roundoff of each other.
pub(super) fn same_matrix(a: &[f64; 16], b: &[f64; 16]) -> bool {
    a.iter().zip(b).all(|(x, y)| (x - y).abs() <= 1e-9)
}

/// `a · b`, column-major 4×4 (`SolidDisplay::pose`'s spelling).
fn mat4_mul_column_major(a: &[f32; 16], b: &[f32; 16]) -> [f32; 16] {
    std::array::from_fn(|k| {
        let (col, row) = (k / 4, k % 4);
        (0..4).map(|n| a[n * 4 + row] * b[col * 4 + n]).sum()
    })
}

/// Move a display rigidly by the row-major `m`: the renderer draws the
/// buffers it holds for the shape through the matrix (`pose`, over any pose
/// already on it), and the positions, normals, edge polylines, vertices and
/// box follow for everything that reads them (picking, sheets, exports).
fn pose_display_by_matrix(display: &mut SolidDisplay, m: &[f64; 16]) {
    let column_major: [f32; 16] = std::array::from_fn(|k| {
        let (col, row) = (k / 4, k % 4);
        m[row * 4 + col] as f32
    });
    display.pose = if display.shape_revision == 0 {
        Some(column_major)
    } else {
        display.pose.map(|before| mat4_mul_column_major(&column_major, &before))
    };
    if display.shape_revision == 0 {
        display.shape_revision = display.revision;
    }
    display.revision = crate::scene::next_revision();
    let point = |p: [f64; 3]| -> [f64; 3] {
        std::array::from_fn(|i| m[i * 4] * p[0] + m[i * 4 + 1] * p[1] + m[i * 4 + 2] * p[2] + m[i * 4 + 3])
    };
    let vector = |v: [f64; 3]| -> [f64; 3] { std::array::from_fn(|i| m[i * 4] * v[0] + m[i * 4 + 1] * v[1] + m[i * 4 + 2] * v[2]) };
    let mut bbox = crate::camera::Aabb::empty();
    for position in &mut display.mesh.positions {
        let posed = point(position.map(f64::from));
        *position = posed.map(|v| v as f32);
        bbox.expand(posed);
    }
    for normal in &mut display.mesh.normals {
        let rotated = vector(normal.map(f64::from));
        let length = rotated.iter().map(|v| v * v).sum::<f64>().sqrt().max(1e-20);
        *normal = rotated.map(|v| (v / length) as f32);
    }
    for edge in &mut display.edges {
        for p in &mut edge.polyline {
            let posed = point(p.map(f64::from));
            *p = posed.map(|v| v as f32);
            bbox.expand(posed);
        }
    }
    for vertex in &mut display.vertices {
        vertex.position = point(vertex.position);
        bbox.expand(vertex.position);
    }
    if !bbox.is_empty() {
        display.bbox = bbox;
    }
}
