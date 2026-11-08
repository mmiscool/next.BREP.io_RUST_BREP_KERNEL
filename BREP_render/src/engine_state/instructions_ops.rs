//! Manufacturing instructions on the engine: an instructions document places
//! its target assembly as one grounded component (the WRAPPER) and keeps what
//! the author writes in its `instructions` block (`brep_instructions::model`).
//!
//! What this module adds on top of the crate:
//! * [`EngineState::instructions_start`]: turn an empty document into
//!   instructions for an assembly;
//! * [`EngineState::instructions_level`]: the wrapper's assembly as the
//!   wizard's occurrence tree, read from its embedded document;
//! * [`EngineState::instructions_run_wizard`]: propose the step tree and
//!   give each step its own PMI view: an iso camera fitted to what the step
//!   shows, the parts not placed yet hidden, and a balloon on each added
//!   part that has a Find Number (a placement inside the wrapper, so the
//!   balloon names its occurrence);
//! * [`EngineState::instructions_restructure`]: an edit that changes which
//!   step adds what (merge, split, insert, reorder, move a part, delete), with
//!   the step views brought up to date in the same undo step;
//! * [`EngineState::instructions_edit`]: every other edit, read-modify-write
//!   of the block as one undo step;
//! * [`EngineState::instructions_show_step`]: show a step's view.
//! * [`EngineState::instructions_fold_retarget`]: a new state of the
//!   assembly re-targets the steps in the same undo step, waiting for
//!   [`EngineState::instructions_retarget_approve`].
//!
//! The wizard reads the scene, so right after a start it waits for the run
//! that builds the wrapper ([`EngineState::instructions_run_wizard_when_built`]).

use super::assembly_ops::ComponentInsert;
use super::EngineState;
use super::{InstructionsRefField, RefSelectTarget};
use brep_instructions::model::{Arrow, Instructions, Step, Style, Target, BLOCK_KEY, DOCUMENT_CLASS};
use brep_instructions::wizard::Level;
use brep_instructions::{edit, retarget, source, wizard};
use brep_kernel::{PmiAnnotation, PmiCamera, PmiDisplay, PmiProjection, PmiView};
use serde_json::{json, Value};

/// The ghost of a part an earlier step placed: neutral grey, see-through.
const GHOST_RGBA: [f32; 4] = [0.62, 0.64, 0.66, 0.28];

/// How long a step's parts take to come home along their insertion arrows.
const INSERTION_SECONDS: f64 = 0.8;
/// The viewport a generated step camera is fitted in (the player data's
/// default), so the fit does not depend on the window the edit was made in.
const GENERATED_VIEWPORT: [f64; 2] = [1280.0, 800.0];

/// A step's insertion motion in the in-app player: its arrowed parts start
/// out along their arrows and move home. Display poses only (the explode
/// machinery's), never a write to the document.
#[derive(Debug, Clone, PartialEq)]
pub struct Insertion {
    /// The view it plays in; showing any other puts it away.
    view: String,
    /// The solids under each arrow and the arrow's start offset.
    moves: Vec<(Vec<String>, [f64; 3])>,
    /// 0 = at the arrows' start, 1 = home.
    fraction: f64,
    /// Held where it is (automation's still pictures): ticks leave it.
    held: bool,
}

/// Where the player goes.
#[derive(Debug, Clone, PartialEq)]
pub enum PlayTo {
    First,
    Next,
    Back,
    /// The shown step again, its insertion from the start.
    Replay,
    Step(String),
}

/// The document key of the document class (`documentClass`).
const DOCUMENT_CLASS_KEY: &str = "documentClass";

impl EngineState {
    /// The document's instructions block; `Ok(None)` when it has none.
    pub fn instructions(&self) -> Result<Option<Instructions>, String> {
        match self.history.document_block(BLOCK_KEY) {
            None | Some(Value::Null) => Ok(None),
            Some(block) => Instructions::from_value(block).map(Some),
        }
    }

    /// Whether the document is an instructions document.
    pub fn is_instructions(&self) -> bool {
        self.history.document_block(DOCUMENT_CLASS_KEY).and_then(Value::as_str) == Some(DOCUMENT_CLASS)
            || self.history.document_block(BLOCK_KEY).is_some()
    }

    pub(super) fn instructions_required(&self) -> Result<Instructions, String> {
        self.instructions()?.ok_or_else(|| "this document is not manufacturing instructions".to_string())
    }

    /// Read the block, apply `edit`, validate and write it back as ONE undo
    /// step (`coalesce` groups a typing run). Nothing is written on a refusal.
    pub fn instructions_edit<T>(
        &mut self,
        coalesce: Option<&str>,
        edit: impl FnOnce(&mut Instructions) -> Result<T, String>,
    ) -> Result<T, String> {
        let mut block = self.instructions_required()?;
        let out = edit(&mut block)?;
        block.validate()?;
        self.history.set_document_block(BLOCK_KEY, Some(block.to_value()), coalesce);
        Ok(out)
    }

    /// The wrapper component: `(id, part name)` of the first placement.
    pub fn instructions_wrapper(&self) -> Result<(String, String), String> {
        self.history
            .features()
            .iter()
            .find(|f| super::components::is_acomp_feature_type(f["type"].as_str().unwrap_or_default()))
            .and_then(|f| Some((f["inputParams"]["id"].as_str()?.to_string(), f["inputParams"]["partName"].as_str()?.to_string())))
            .ok_or_else(|| "the instructions place no assembly".to_string())
    }

    /// The wrapper's assembly as the wizard's occurrence tree.
    pub fn instructions_level(&self) -> Result<Level, String> {
        let (id, part) = self.instructions_wrapper()?;
        let document = &self.history.parts_library()[part.as_str()]["document"];
        if !source::has_placements(document) {
            return Err(format!("'{part}' is not an assembly: instructions need one to describe"));
        }
        Ok(source::level_from_document(document, &id, "", &part))
    }

    /// Make this EMPTY document the instructions for an assembly: place it as
    /// the grounded wrapper, mark the document class and start a block for
    /// `target`. One undo step. Returns the wrapper's id.
    pub fn instructions_start(&mut self, insert: ComponentInsert<'_>, target: Target) -> Result<String, String> {
        if !self.history.features().is_empty() || self.history.document_block(BLOCK_KEY).is_some() {
            return Err("instructions start from an empty document".into());
        }
        if let ComponentInsert::New { document_json, .. } = &insert {
            let document: Value = serde_json::from_str(document_json).map_err(|e| format!("the assembly is not a document: {e}"))?;
            if !source::has_placements(&document) {
                return Err("instructions need an assembly to describe".into());
            }
        }
        // The first placement is inserted grounded.
        let id = self.insert_component(insert)?;
        self.history.set_document_block_no_undo(DOCUMENT_CLASS_KEY, Some(Value::String(DOCUMENT_CLASS.into())));
        self.history.set_document_block_no_undo(BLOCK_KEY, Some(Instructions::new(target).to_value()));
        Ok(id)
    }

    /// Propose the step tree from the wrapper's assembly, replacing the
    /// block's steps (the target, template and front matter stay), and
    /// give each step its view. The views the old steps had are removed. One
    /// undo step. Returns the step count.
    pub fn instructions_run_wizard(&mut self) -> Result<usize, String> {
        let mut block = self.instructions_required()?;
        let level = self.instructions_level()?;
        let old_views: Vec<String> = block.steps().filter_map(|(_, s)| s.view.clone()).collect();
        wizard::propose(&mut block, &level);
        let mut state = self.pmi_state();
        state.views.retain(|view| !old_views.contains(&view.id));
        self.instructions_step_views(&mut block, &mut state)?;
        block.validate()?;
        let steps = block.step_count();
        self.history.set_document_block(BLOCK_KEY, Some(block.to_value()), None);
        let pmi = if state.is_empty() { None } else { serde_json::to_value(&state).ok() };
        self.history.set_pmi_block_no_undo(pmi);
        self.rerun_history();
        Ok(steps)
    }

    /// [`instructions_run_wizard`](Self::instructions_run_wizard) now when
    /// the scene is current, else once the pending run lands (a background
    /// runner has not built the wrapper's solids yet right after
    /// [`instructions_start`](Self::instructions_start)).
    pub fn instructions_run_wizard_when_built(&mut self) -> Result<(), String> {
        self.instructions_required()?;
        if self.run_pending() {
            self.instructions_wizard_pending = true;
            return Ok(());
        }
        self.instructions_run_wizard().map(|_| ())
    }

    /// Whether the wizard is waiting for a run to land.
    pub fn instructions_wizard_pending(&self) -> bool {
        self.instructions_wizard_pending
    }

    /// A structural edit (merge, split, insert, reorder, move a part, delete):
    /// `edit` on the block, then every step's view brought up to date: the
    /// parts hidden and the balloons follow the steps' parts, each step keeps
    /// its camera and the author's own annotations, a step without a view
    /// gets one, and the views of steps that went away are removed. One undo
    /// step.
    pub fn instructions_restructure<T>(&mut self, edit: impl FnOnce(&mut Instructions) -> Result<T, String>) -> Result<T, String> {
        self.instructions_restructure_with_views(|block, _| edit(block))
    }

    /// [`Self::instructions_restructure`] whose edit also reaches the PMI
    /// state (a step's view camera).
    fn instructions_restructure_with_views<T>(
        &mut self,
        edit: impl FnOnce(&mut Instructions, &mut brep_kernel::PmiState) -> Result<T, String>,
    ) -> Result<T, String> {
        let mut block = self.instructions_required()?;
        let before: Vec<String> = block.steps().filter_map(|(_, s)| s.view.clone()).collect();
        let mut state = self.pmi_state();
        let out = edit(&mut block, &mut state)?;
        block.validate()?;
        let kept: Vec<String> = block.steps().filter_map(|(_, s)| s.view.clone()).collect();
        state.views.retain(|view| !before.contains(&view.id) || kept.contains(&view.id));
        self.instructions_step_views(&mut block, &mut state)?;
        self.history.set_document_block(BLOCK_KEY, Some(block.to_value()), None);
        let pmi = if state.is_empty() { None } else { serde_json::to_value(&state).ok() };
        self.history.set_pmi_block_no_undo(pmi);
        self.rerun_history();
        Ok(out)
    }

    /// Fold a new state of the wrapper's assembly into the steps of the
    /// `incoming` document before it is adopted (Update components, a linked
    /// tab's newer state), so the assembly and the re-target are one undo
    /// step: the wizard reruns on the changed parts only and the change waits
    /// in the block's `retarget` for the author's approval. An assembly whose
    /// occurrences did not change only moves the target. Refused while a
    /// re-target waits. Returns whether the steps changed (their views are
    /// made once the new revision is built).
    pub(super) fn instructions_fold_retarget(&self, incoming: &mut Value) -> Result<bool, String> {
        let Ok((id, part)) = self.instructions_wrapper() else { return Ok(false) };
        let old_entry = &self.history.parts_library()[part.as_str()];
        let new_entry = &incoming["partsLibrary"][part.as_str()];
        if new_entry.is_null() || old_entry["document"] == new_entry["document"] {
            return Ok(false);
        }
        let Some(mut block) = incoming.get(BLOCK_KEY).filter(|b| !b.is_null()).map(Instructions::from_value).transpose()? else {
            return Ok(false);
        };
        if block.retarget.is_some() {
            return Err("the instructions' re-target waits for approval: approve it in the Instructions pane, then update again".into());
        }
        let old = source::level_from_document(&old_entry["document"], &id, "", &part);
        let new = source::level_from_document(&new_entry["document"], &id, "", &part);
        let text = |key: &str, or: &str| new_entry[key].as_str().filter(|v| !v.is_empty()).unwrap_or(or).to_string();
        let to = Target {
            source_key: text("sourceKey", &block.target.source_key),
            signature: text("sourceSignature", &block.target.signature),
            ..block.target.clone()
        };
        let diff = retarget::diff(&old, &new);
        let changed = !diff.is_empty();
        if changed {
            let before: Vec<String> = block.steps().filter_map(|(_, s)| s.view.clone()).collect();
            retarget::apply(&mut block, &new, &diff, to)?;
            // The views of steps a dropped sub-assembly's group took away.
            let kept: Vec<String> = block.steps().filter_map(|(_, s)| s.view.clone()).collect();
            if let Some(views) = incoming["pmi"]["views"].as_array_mut() {
                views.retain(|view| view["id"].as_str().is_none_or(|id| !before.iter().any(|b| b == id) || kept.iter().any(|k| k == id)));
            }
        } else {
            block.target = to;
        }
        block.validate()?;
        incoming[BLOCK_KEY] = block.to_value();
        Ok(changed)
    }

    /// Bring every step's view up to date with the block without an undo step
    /// of its own (it belongs to the edit that changed the steps): what each
    /// hides and its balloons follow the steps' parts, and a new step gets a
    /// view fitted to the solids the run just built.
    pub(super) fn instructions_refresh_views(&mut self) -> Result<(), String> {
        let mut block = self.instructions_required()?;
        let mut state = self.pmi_state();
        self.instructions_step_views(&mut block, &mut state)?;
        self.history.set_document_block_no_undo(BLOCK_KEY, Some(block.to_value()));
        let pmi = if state.is_empty() { None } else { serde_json::to_value(&state).ok() };
        self.history.set_pmi_block_no_undo(pmi);
        self.rerun_history();
        Ok(())
    }

    /// Approve the pending re-target: the new revision becomes the target and
    /// the steps it left empty go, with their views. One undo step. Returns
    /// the new target.
    pub fn instructions_retarget_approve(&mut self) -> Result<Target, String> {
        self.instructions_restructure(retarget::approve)
    }

    /// Reject the pending re-target: the ONE undo that takes the new revision
    /// and its re-target back together (what Undo does), offered as a button
    /// and a command. Refused when none waits; when the re-target came in with
    /// the file (nothing in this session takes it back: approve it, or update
    /// again later); and when a later edit sits on top of that undo step (it
    /// would go too: undo it first, or approve). The newest step is the
    /// re-target's when the block as it stood before it has no `retarget`.
    /// Returns the target the instructions keep describing.
    pub fn instructions_retarget_reject(&mut self) -> Result<Target, String> {
        let block = self.instructions_required()?;
        let from = block.retarget.as_ref().ok_or("no re-target is waiting for approval")?.from.clone();
        if !self.history.can_undo() {
            return Err("the re-target came with the file: nothing in this session takes it back; approve it, or update again later".into());
        }
        let fold_is_newest = self
            .history
            .undo_top_block(BLOCK_KEY)
            .is_some_and(|before| before.get("retarget").is_none_or(Value::is_null));
        if !fold_is_newest {
            return Err("edits followed the re-target: undo them first, or approve it".into());
        }
        self.undo();
        Ok(from)
    }

    /// Enter reference selection for step `step_id`'s occurrence `field` (an
    /// arrow's part or a callout's): the field takes ONE solid of the placed
    /// assembly, picked in the viewport, and Finish derives its occurrence
    /// (`Self::instructions_commit_ref`). `seed` is the occurrence it holds
    /// now, lit as the pick's seed through its first solid. Nothing rolls.
    pub fn begin_ref_select_for_instructions(&mut self, step_id: &str, field: InstructionsRefField, seed: Option<&str>) {
        let restore_index = self.history.rollback();
        let filter = vec!["SOLID".to_string()];
        let label = match field {
            InstructionsRefField::Arrow(_) => "Part the arrow moves".to_string(),
            InstructionsRefField::Callout(_) => "Part the callout points at".to_string(),
        };
        let path = match field {
            InstructionsRefField::Arrow(index) => vec!["arrows".to_string(), index.to_string(), "occurrence".to_string()],
            InstructionsRefField::Callout(index) => vec!["callouts".to_string(), index.to_string(), "occurrence".to_string()],
        };
        let names = seed.and_then(|occurrence| self.instructions_occurrence_solid(occurrence)).into_iter().collect();
        self.selection_filter = crate::engine_state::selection_filter::SelectionFilter::from_ref_filter(&filter);
        self.ref_select = Some(super::RefSelectState {
            feature_id: step_id.to_string(),
            path,
            label,
            filter,
            multiple: false,
            names,
            restore_index,
            target: RefSelectTarget::Instructions { field },
            picks: 0,
            claimed: false,
            finish_requested: false,
        });
        self.sync_ref_select_emphasis();
    }

    /// The occurrence a picked solid belongs to: its name without the member
    /// (`ACOMP1:ACOMP3:Pin` → `ACOMP1:ACOMP3`), when the placed assembly
    /// holds that occurrence.
    pub fn instructions_occurrence_of_solid(&self, solid: &str) -> Result<String, String> {
        let (occurrence, _) = solid.rsplit_once(':').ok_or_else(|| format!("'{solid}' is not a part of the assembly"))?;
        let level = self.instructions_level()?;
        if !wizard::flatten(&level).iter().any(|(_, c)| c.path == occurrence) {
            return Err(format!("'{solid}' is not a part of the assembly"));
        }
        Ok(occurrence.to_string())
    }

    /// The first drawn solid of `occurrence` (`<occurrence>:<member>`), for
    /// lighting the occurrence in the viewport.
    pub fn instructions_occurrence_solid(&self, occurrence: &str) -> Option<String> {
        let prefix = format!("{occurrence}:");
        self.scene.solids().iter().map(|solid| solid.name.clone()).find(|name| name.starts_with(&prefix))
    }

    /// Write a finished occurrence pick into step `step_id`'s `field`: an
    /// arrow takes a part the step adds (any other is refused, the arrow
    /// unchanged), a callout any part of the assembly. One undo step, the
    /// step views following.
    pub(crate) fn instructions_commit_ref(&mut self, step_id: &str, field: InstructionsRefField, solid: &str) -> Result<(), String> {
        let occurrence = self.instructions_occurrence_of_solid(solid)?;
        let step_id = step_id.to_string();
        self.instructions_edit(None, |block| {
            let step = block.step_mut(&step_id).ok_or_else(|| format!("no step '{step_id}'"))?;
            match field {
                InstructionsRefField::Arrow(index) => {
                    if !step.adds.iter().any(|add| add.occurrences.contains(&occurrence)) {
                        return Err(format!("{occurrence} is not a part this step adds: an arrow moves one of the step's own parts"));
                    }
                    let arrow = step.arrows.get_mut(index).ok_or_else(|| format!("no arrow {index}"))?;
                    arrow.occurrence = occurrence;
                }
                InstructionsRefField::Callout(index) => {
                    let callout = step.callouts.get_mut(index).ok_or_else(|| format!("no callout {index}"))?;
                    callout.occurrence = Some(occurrence);
                }
            }
            Ok(())
        })?;
        self.history.break_coalescing();
        Ok(())
    }

    /// A new empty step after `after` (or at the end), with its own view.
    /// Returns its id.
    pub fn instructions_new_step(&mut self, after: Option<&str>) -> Result<String, String> {
        self.instructions_restructure(|block| edit::insert_step(block, after))
    }

    /// A BOM row dragged into a step: add `occurrences` of the wrapper's
    /// assembly to `step_id` ([`edit::add_parts`]). An occurrence not in the
    /// assembly, or a step that does not exist, is refused with nothing
    /// written. The step views follow in the same undo step. Returns what
    /// was added and what was left where it is.
    pub fn instructions_add_parts(&mut self, step_id: &str, occurrences: &[String]) -> Result<edit::AddReport, String> {
        let (wrapper, _) = self.instructions_wrapper()?;
        let level = self.instructions_level()?;
        let flat = wizard::flatten(&level);
        let mut components = Vec::new();
        for occurrence in occurrences {
            if *occurrence == wrapper {
                return Err("that row is the whole assembly: drag a part row into the step".into());
            }
            let component = flat
                .iter()
                .find(|(_, c)| &c.path == occurrence)
                .map(|(_, c)| *c)
                .ok_or_else(|| format!("'{occurrence}' is not an occurrence of the assembly"))?;
            components.push(component);
        }
        if self.instructions_required()?.step(step_id).is_none() {
            return Err(format!("no step '{step_id}'"));
        }
        self.instructions_restructure(|block| edit::add_parts(block, step_id, &components))
    }

    /// Every occurrence of the part named `part` (its name or its PLM part id) at the assembly's top level
    /// (what a packed BOM row stands for; a sub-assembly's own parts belong
    /// to its group) into `step_id`, the ones already in a step reported.
    /// Refused for a part the top level does not have.
    pub fn instructions_add_part(&mut self, step_id: &str, part: &str) -> Result<edit::AddReport, String> {
        let level = self.instructions_level()?;
        let occurrences: Vec<String> = level.components.iter().filter(|c| c.part == part || c.part_key() == part).map(|c| c.path.clone()).collect();
        if occurrences.is_empty() {
            return Err(format!("the assembly has no part '{part}'"));
        }
        self.instructions_add_parts(step_id, &occurrences)
    }

    /// Take a part out of a step ([`edit::remove_add`]): it is not planned
    /// any more until it is added again. One undo step, views following.
    pub fn instructions_remove_add(&mut self, step_id: &str, add_index: usize) -> Result<(), String> {
        self.instructions_restructure(|block| edit::remove_add(block, step_id, add_index).map(drop))
    }

    /// Move a step under `parent` (`None`: the top level) at `index`, its
    /// children with it ([`edit::nest_step`]). One undo step, views following
    /// (a nested step's camera looks from its new parent's direction).
    pub fn instructions_nest_step(&mut self, step_id: &str, parent: Option<&str>, index: usize) -> Result<(), String> {
        self.instructions_restructure(|block| edit::nest_step(block, step_id, parent, index))
    }

    /// The parts of the assembly not yet in any step, as `(occurrence, part)`
    /// ([`edit::unplanned`]); empty when the document is not instructions.
    pub fn instructions_unplanned(&self) -> Vec<(String, String)> {
        let Ok(Some(block)) = self.instructions() else { return Vec::new() };
        let Ok(level) = self.instructions_level() else { return Vec::new() };
        edit::unplanned(&block, &level).into_iter().map(|c| (c.path.clone(), c.part.clone())).collect()
    }

    /// Capture the viewport's camera as `step_id`'s view camera. It is the
    /// author's from then on: every edit keeps it, until
    /// [`Self::instructions_reset_view`]. One undo step.
    pub fn instructions_capture_camera(&mut self, step_id: &str) -> Result<(), String> {
        let snapshot = self.snapshot_camera();
        self.instructions_restructure_with_views(|block, state| {
            let step = block.step_mut(step_id).ok_or_else(|| format!("no step '{step_id}'"))?;
            step.camera_captured = true;
            if let Some(view) = step.view.as_deref().and_then(|id| state.find_view_mut(id)) {
                // A section saved before its plane was stored is anchored on
                // the old camera first, as the PMI workbench's Update camera
                // does (no step view carries a section today).
                if let Some(section) = view.display.section.as_mut() {
                    section.freeze(view.camera.as_ref());
                }
                view.camera = Some(snapshot);
            }
            Ok(())
        })
    }

    /// Drop `step_id`'s captured camera: its view's camera is generated again
    /// from the step's parts and its parent's direction. One undo step.
    pub fn instructions_reset_view(&mut self, step_id: &str) -> Result<(), String> {
        self.instructions_restructure(|block| {
            block.step_mut(step_id).ok_or_else(|| format!("no step '{step_id}'"))?.camera_captured = false;
            Ok(())
        })
    }

    /// The step whose view `view_id` is, if any.
    pub(super) fn instructions_step_of_view(&self, view_id: &str) -> Option<String> {
        let block = self.instructions().ok().flatten()?;
        let step = block.steps().find(|(_, step)| step.view.as_deref() == Some(view_id)).map(|(_, step)| step.id.clone());
        step
    }

    /// Give every step of `block` its view in `state`: the parts not placed
    /// yet hidden and a balloon on each added part that has a Find Number (a
    /// placement inside the wrapper, so the balloon names its occurrence).
    /// A step's existing view keeps its id and the annotations the author
    /// added (explode poses among them). Its camera is generated from the
    /// step's parts ([`Self::generated_step_camera`]) unless the author
    /// captured one, which stays until Reset view. Parents go before their
    /// children, whose cameras look from the parent's direction. The user's
    /// camera is put back afterwards.
    fn instructions_step_views(&mut self, block: &mut Instructions, state: &mut brep_kernel::PmiState) -> Result<(), String> {
        self.pmi_deactivate_view();
        let camera = self.camera.clone();
        let solids: Vec<String> = self.scene.solids().iter().map(|s| s.name.clone()).collect();
        let boxes: std::collections::HashMap<String, crate::camera::Aabb> =
            self.scene.solids().iter().map(|s| (s.name.clone(), s.bbox)).collect();
        fn parents_first(steps: &[brep_instructions::model::Step], out: &mut Vec<String>) {
            for step in steps {
                out.push(step.id.clone());
                parents_first(&step.steps, out);
            }
        }
        let mut ids = Vec::new();
        parents_first(&block.steps, &mut ids);
        for step_id in &ids {
            let presentation = edit::presentation(block, step_id, &solids)?;
            let number = block.step_number(step_id).unwrap_or(0);
            let step = block.step(step_id).expect("listed step");
            let existing = step.view.as_deref().and_then(|id| state.find_view(id)).cloned();
            let parent_camera = block
                .parent_id(step_id)
                .flatten()
                .and_then(|parent| block.step(parent))
                .and_then(|parent| parent.view.as_deref())
                .and_then(|id| state.find_view(id))
                .and_then(|view| view.camera.clone());
            // A Line art step whose parts slide in along the derived arrows
            // frames where they start too.
            let slides = block.template.style == Style::LineArt && step.arrows.is_empty();
            let camera_now = match existing.as_ref().and_then(|view| view.camera.clone()).filter(|_| step.camera_captured) {
                Some(kept) => kept,
                None => self.generated_step_camera(&presentation, &boxes, parent_camera.as_ref(), slides),
            };
            // The author's annotations stay; the step's own balloons (the
            // ones naming an occurrence) are made again from its parts.
            let mut annotations: Vec<PmiAnnotation> = existing
                .as_ref()
                .map(|view| {
                    view.annotations
                        .iter()
                        .filter(|a| !(a.kind == "balloon" && a.params.get("occurrence").is_some()) && !a.is_baseline_hide())
                        .cloned()
                        .collect()
                })
                .unwrap_or_default();
            // The parts not placed yet are the view's baseline hide step,
            // made again from the step after its balloons (its id kept); the
            // author's own hide steps stay.
            let baseline_id = existing
                .as_ref()
                .and_then(|view| view.annotations.iter().find(|a| a.is_baseline_hide()))
                .map(|a| a.id().to_string())
                .filter(|id| !id.is_empty());
            let view_id = match &existing {
                Some(view) => view.id.clone(),
                None => state.next_id("VIEW"),
            };
            for add in step.adds.iter().filter(|a| !a.find_number.is_empty()) {
                let Some(occurrence) = add.occurrences.first() else { continue };
                let prefix = format!("{occurrence}:");
                let Some(member) = solids.iter().find(|name| name.starts_with(&prefix)) else { continue };
                annotations.push(PmiAnnotation {
                    plugin_replay: None,
                    persistent_data: Value::Null,
                    kind: "balloon".into(),
                    enabled: true,
                    params: json!({ "id": state.next_id("BAL"), "target": member, "occurrence": occurrence }),
                    label_world: None,
                });
            }
            if !presentation.hidden.is_empty() {
                let id = baseline_id.unwrap_or_else(|| brep_kernel::baseline_hide_id(&view_id));
                annotations.push(PmiAnnotation::hide_step(&id, &presentation.hidden, true));
            }
            let display = existing.as_ref().map(|view| view.display.clone()).unwrap_or_else(|| PmiDisplay::new(12.0, false, None));
            let view = PmiView {
                id: view_id.clone(),
                name: format!("Step {number} · {}", step.title),
                camera: Some(camera_now),
                display,
                annotations,
            };
            match state.find_view_mut(&view_id) {
                Some(slot) => *slot = view,
                None => state.views.push(view),
            }
            block.step_mut(step_id).expect("listed step").view = Some(view_id);
        }
        self.camera = camera;
        Ok(())
    }

    /// A step's generated camera: it frames everything the step shows, the
    /// parts it adds (its own and its children's) and the ghosts placed
    /// before it, so no part runs off the picture. It looks from its parent's
    /// camera direction, with the parent's up and projection, or from the
    /// isometric in the app's default (orthographic) projection at the top
    /// level. The fit is made in a fixed viewport
    /// ([`GENERATED_VIEWPORT`]), so the same step gives the same camera
    /// whatever window the edit was made in. Leaves `self.camera` changed.
    fn generated_step_camera(
        &mut self,
        presentation: &edit::Presentation,
        boxes: &std::collections::HashMap<String, crate::camera::Aabb>,
        parent: Option<&PmiCamera>,
        slides: bool,
    ) -> PmiCamera {
        let union = |names: &[String]| {
            let mut bbox = crate::camera::Aabb::empty();
            for name in names {
                if let Some(solid) = boxes.get(name) {
                    bbox.union(solid);
                }
            }
            bbox
        };
        let mut shown = union(&presentation.placed);
        shown.union(&union(&presentation.adding));
        let frame = shown.clone();
        match parent {
            Some(parent) => {
                let (kind, scale) = match parent.projection {
                    PmiProjection::Orthographic { half_height } => ("orthographic", half_height),
                    PmiProjection::Perspective { fov_y_deg } => ("perspective", fov_y_deg),
                };
                let json = json!({ "kind": kind, "eye": parent.eye, "target": parent.target, "up": parent.up, "scale": scale }).to_string();
                let _ = self.camera.apply_state_json(&json);
            }
            None => {
                // The app's default projection, whatever the viewport is in,
                // so the same step gives the same camera.
                self.camera.projection = crate::view::Projection::Orthographic { half_height: 10.0 };
                self.camera.standard_view("iso");
            }
        }
        let mut frame = frame;
        if slides {
            if let Some(offset) = default_start_offset(presentation, boxes, self.camera.up) {
                let mut start = union(&presentation.adding);
                start.min = std::array::from_fn(|k| start.min[k] + offset[k]);
                start.max = std::array::from_fn(|k| start.max[k] + offset[k]);
                frame.union(&start);
            }
        }
        self.camera.width = GENERATED_VIEWPORT[0];
        self.camera.height = GENERATED_VIEWPORT[1];
        if !frame.is_empty() {
            self.camera.zoom_to_fit(&frame, 1.25);
        }
        // An orthographic fit keeps the eye where it was: put it at a fixed
        // distance (clear of everything shown) so it depends on the frame
        // alone, not on the viewport the edit was made in.
        if matches!(self.camera.projection, crate::view::Projection::Orthographic { .. }) {
            // An empty step shows nothing: its box is inside out (+inf..-inf),
            // whose infinite diagonal put the eye at infinity, and the history
            // document then stored it as null and refused every later run.
            let size = if shown.is_empty() { [0.0; 3] } else { shown.size() };
            let diagonal = (size[0] * size[0] + size[1] * size[1] + size[2] * size[2]).sqrt();
            let distance = (4.0 * diagonal).max(10.0);
            let (eye, target) = (self.camera.eye, self.camera.target);
            let direction = [eye[0] - target[0], eye[1] - target[1], eye[2] - target[2]];
            let length = (direction[0] * direction[0] + direction[1] * direction[1] + direction[2] * direction[2]).sqrt();
            if length > 1e-12 {
                self.camera.eye = [
                    target[0] + direction[0] / length * distance,
                    target[1] + direction[1] / length * distance,
                    target[2] + direction[2] / length * distance,
                ];
            }
        }
        self.snapshot_camera()
    }

    /// Show a step's view (its camera, hidden parts and balloons).
    pub fn instructions_show_step(&mut self, step_id: &str) -> Result<(), String> {
        let block = self.instructions_required()?;
        let step = block.step(step_id).ok_or_else(|| format!("no step '{step_id}'"))?;
        let view = step.view.clone().ok_or_else(|| format!("step '{step_id}' has no view: run the wizard or pick one"))?;
        self.pmi_activate_view(&view)
    }

    /// Draw the shown step as the players do. In a Line art document the
    /// scene carries the look ([`crate::scene::RenderScene::line_art`]): the
    /// placed parts stay in it, drawn white with grey lines, the added ones
    /// flat blue with black lines. Otherwise the parts earlier steps placed
    /// are ghosted (grey and translucent, in the context layer, which draws
    /// without depth writes or picks) and the parts the step adds solid. Run
    /// after a view is applied and after every run while one is shown (the
    /// copies follow the rebuilt displays); with no step shown, or before a
    /// view is put away, the ghosts go and their parts show again. Only an
    /// instructions document's context layer is touched: a Work part keeps
    /// its context on the part's own engine.
    pub(crate) fn instructions_refresh_ghosts(&mut self) {
        if !self.is_instructions() {
            return;
        }
        let shown = self.instructions_shown_presentation();
        // A Line art step draws its placed parts opaque and white in the
        // scene itself (`RenderScene::line_art`), not as translucent ghosts:
        // hidden lines must not show through them.
        let line_art = shown.is_some() && self.instructions().ok().flatten().is_some_and(|block| block.template.style == Style::LineArt);
        let all_placed = shown.map(|presentation| presentation.placed).unwrap_or_default();
        let look = line_art.then(|| crate::scene::LineArtLook { placed: all_placed.iter().cloned().collect() });
        if self.scene.line_art != look {
            self.scene.line_art = look;
            self.dirty = true;
        }
        let placed = if line_art { Vec::new() } else { all_placed };
        let gone: Vec<String> =
            self.context_scene.solids().iter().map(|ghost| ghost.name.clone()).filter(|name| !placed.contains(name)).collect();
        for name in gone {
            self.context_scene.remove_solid(&name);
            self.scene.set_visible(&name, true);
            self.dirty = true;
        }
        for name in placed {
            let Some(solid) = self.scene.solid(&name) else { continue };
            // A hidden part without a ghost is one the view hides; one with a
            // ghost was ghosted before, and a run may have rebuilt it since.
            if !solid.visible && self.context_scene.solid(&name).is_none() {
                continue;
            }
            let mut ghost = solid.clone();
            ghost.visible = true;
            ghost.source_handle = 0;
            ghost.vertices.clear();
            if self.context_scene.solid(&name) != Some(&ghost) {
                self.context_scene.insert_solid(ghost);
                self.dirty = true;
            }
            if solid.visible {
                self.scene.set_visible(&name, false);
                self.dirty = true;
            }
        }
        if !self.context_scene.solids().is_empty() && self.context_scene.tint != Some(GHOST_RGBA) {
            self.context_scene.tint = Some(GHOST_RGBA);
            self.dirty = true;
        }
    }

    /// The arrows `step_id`'s parts come home along: the step's own; in a
    /// Line art document, a step without any gets one per part it adds (its
    /// own and its group's), all travelling down its view's up direction
    /// from a start clear of the parts already placed
    /// (`default_start_offset`). Derived each time, never written to the
    /// document: the author's arrows replace them. The first step of a
    /// group, which places nothing over anything, slides nothing in.
    pub fn instructions_step_arrows(&self, block: &Instructions, step_id: &str) -> Vec<Arrow> {
        let Some(step) = block.step(step_id) else { return Vec::new() };
        if !step.arrows.is_empty() || block.template.style != Style::LineArt {
            return step.arrows.clone();
        }
        let pmi = self.pmi_state();
        let Some(up) = step.view.as_deref().and_then(|id| pmi.find_view(id)).and_then(|view| view.camera.as_ref()).map(|camera| camera.up) else {
            return Vec::new();
        };
        let solids: Vec<String> = self.scene.solids().iter().map(|s| s.name.clone()).collect();
        let Ok(presentation) = edit::presentation(block, step_id, &solids) else { return Vec::new() };
        let boxes: std::collections::HashMap<String, crate::camera::Aabb> =
            self.scene.solids().iter().map(|s| (s.name.clone(), s.bbox)).collect();
        default_arrows(step, &presentation, &boxes, up)
    }

    /// Play a step in the in-app player: show its view (its camera, ghosts
    /// and balloons) and start its parts' insertion along their arrows.
    /// Steps go in reading order (a step's children before it); Next on
    /// the last step and Back on the first stay put. Returns the step shown.
    pub fn instructions_play(&mut self, to: PlayTo) -> Result<String, String> {
        let block = self.instructions_required()?;
        let order: Vec<String> = block.steps().map(|(_, step)| step.id.clone()).collect();
        if order.is_empty() {
            return Err("there are no steps yet: run the wizard first".into());
        }
        let shown = self.instructions_shown_step();
        let at = shown.as_ref().and_then(|id| order.iter().position(|step| step == id));
        let step = match to {
            PlayTo::First => order[0].clone(),
            PlayTo::Next => order[at.map_or(0, |i| (i + 1).min(order.len() - 1))].clone(),
            PlayTo::Back => order[at.map_or(0, |i| i.saturating_sub(1))].clone(),
            PlayTo::Replay => shown.unwrap_or_else(|| order[0].clone()),
            PlayTo::Step(id) => order.iter().find(|step| **step == id).cloned().ok_or_else(|| format!("no step '{id}'"))?,
        };
        self.instructions_show_step(&step)?;
        let view = self.pmi_active_view().map(str::to_string).unwrap_or_default();
        let solids: Vec<String> = self.scene.solids().iter().map(|solid| solid.name.clone()).collect();
        let presentation = edit::presentation(&block, &step, &solids)?;
        let under = |path: &str, occurrence: &str| path == occurrence || (path.starts_with(occurrence) && path[occurrence.len()..].starts_with(':'));
        let arrows = self.instructions_step_arrows(&block, &step);
        let moves: Vec<(Vec<String>, [f64; 3])> = block
            .step(&step)
            .map(|_| {
                arrows
                    .iter()
                    .map(|arrow| {
                        // Only the parts this step adds move: a placed part's
                        // display is a ghost's source and stays where it is.
                        let names: Vec<String> = presentation.adding.iter().filter(|name| under(name, &arrow.occurrence)).cloned().collect();
                        (names, arrow.start_offset())
                    })
                    .filter(|(names, offset)| !names.is_empty() && offset.iter().any(|v| *v != 0.0))
                    .collect()
            })
            .unwrap_or_default();
        self.instructions_insertion = (!moves.is_empty()).then(|| Insertion { view, moves, fraction: 0.0, held: false });
        self.pmi_apply_explode();
        self.refresh_pmi_overlay();
        Ok(step)
    }

    /// Advance the insertion by `dt` seconds; `true` while another frame is
    /// needed. The app calls it once per frame.
    pub fn instructions_insertion_tick(&mut self, dt: f64) -> bool {
        let Some(insertion) = self.instructions_insertion.as_mut() else { return false };
        if self.pmi_active_view.as_deref() != Some(insertion.view.as_str()) {
            self.instructions_insertion = None;
            return false;
        }
        if insertion.held {
            return false;
        }
        insertion.fraction = (insertion.fraction + dt.max(0.0) / INSERTION_SECONDS).min(1.0);
        let done = insertion.fraction >= 1.0;
        if done {
            self.instructions_insertion = None;
        }
        self.pmi_apply_explode();
        self.refresh_pmi_overlay();
        !done
    }

    /// The insertion's display poses for [`Self::pmi_apply_explode`]:
    /// `(solids, translate, rotate°, scale, centre)`, empty unless the view
    /// it plays in is the one shown.
    pub(crate) fn instructions_insertion_poses(&self) -> Vec<(Vec<String>, [f64; 3], [f64; 3], [f64; 3], [f64; 3])> {
        let Some(insertion) = &self.instructions_insertion else { return Vec::new() };
        if self.pmi_active_view.as_deref() != Some(insertion.view.as_str()) {
            return Vec::new();
        }
        let left = 1.0 - insertion.fraction;
        insertion
            .moves
            .iter()
            .map(|(names, offset)| (names.clone(), offset.map(|v| v * left), [0.0; 3], [1.0; 3], [0.0; 3]))
            .collect()
    }

    /// The insertion's arrows while it plays, in world space: for each
    /// moving group of parts, from the edge of its screen outline where it is
    /// now (on its centre line, on the side it travels toward) to where that
    /// point lands at home (its seat), so the arrow never crosses the parts,
    /// shortens as they come in and is gone once they are seated.
    pub(crate) fn instructions_insertion_arrows(&self) -> Vec<([f64; 3], [f64; 3])> {
        let Some(insertion) = &self.instructions_insertion else { return Vec::new() };
        if self.pmi_active_view.as_deref() != Some(insertion.view.as_str()) {
            return Vec::new();
        }
        let left = 1.0 - insertion.fraction;
        insertion
            .moves
            .iter()
            .filter_map(|(names, offset)| {
                let mut bbox = crate::camera::Aabb::empty();
                for name in names {
                    let home = self.pmi_explode_originals.get(name).or_else(|| self.scene.solid(name));
                    if let Some(display) = home {
                        bbox.union(&display.bbox);
                    }
                }
                self.outline_arrow(&bbox, offset.map(|v| v * left))
            })
            .collect()
    }

    /// Hold the playing insertion at `fraction` (clamped to 0..1) until
    /// another step is played. Nothing when no part is moving.
    pub fn instructions_hold_insertion(&mut self, fraction: f64) {
        let Some(insertion) = self.instructions_insertion.as_mut() else { return };
        insertion.fraction = if fraction.is_finite() { fraction.clamp(0.0, 1.0) } else { 0.0 };
        insertion.held = true;
        self.pmi_apply_explode();
        self.refresh_pmi_overlay();
    }

    /// An insertion or explode arrow for parts whose bounds at home are
    /// `home` and which stand `now` away from home: from the edge of their
    /// screen outline (on their centre line, on the side they travel toward)
    /// to where that point lands at home, so it never crosses the parts and
    /// is exactly the travel left. `None` with nothing left to travel.
    pub(crate) fn outline_arrow(&self, home: &crate::camera::Aabb, now: [f64; 3]) -> Option<([f64; 3], [f64; 3])> {
        let length = (now[0] * now[0] + now[1] * now[1] + now[2] * now[2]).sqrt();
        if home.is_empty() || !(length > 1e-9) {
            return None;
        }
        // The travel direction, toward home.
        let d = now.map(|v| -v / length);
        let (c, size) = (home.center(), home.size());
        let h = 0.5 * (size[0] * d[0].abs() + size[1] * d[1].abs() + size[2] * d[2].abs());
        let (right, up, _) = self.camera.basis();
        let screen = |p: [f64; 3]| [crate::geometry3d::dot3(p, right), crate::geometry3d::dot3(p, up)];
        let sd = screen(d);
        let sl = (sd[0] * sd[0] + sd[1] * sd[1]).sqrt();
        let mut lead = h;
        if sl > 1e-9 {
            let u = [sd[0] / sl, sd[1] / sl];
            let along = |p: [f64; 3]| {
                let s = screen(p);
                s[0] * u[0] + s[1] * u[1]
            };
            let reach = (0..8)
                .map(|i| {
                    along([
                        if i & 1 == 0 { home.min[0] } else { home.max[0] },
                        if i & 2 == 0 { home.min[1] } else { home.max[1] },
                        if i & 4 == 0 { home.min[2] } else { home.max[2] },
                    ])
                })
                .fold(f64::NEG_INFINITY, f64::max);
            lead = ((reach - along(c)) / sl).max(h);
        }
        let seat: [f64; 3] = std::array::from_fn(|k| c[k] + d[k] * lead);
        Some((std::array::from_fn(|k| seat[k] + now[k]), seat))
    }

    /// How far the shown step's parts have come home (0..1), while they move.
    pub fn instructions_insertion_fraction(&self) -> Option<f64> {
        self.instructions_insertion.as_ref().map(|insertion| insertion.fraction)
    }

    /// The step whose view is shown.
    pub fn instructions_shown_step(&self) -> Option<String> {
        let active = self.pmi_active_view()?;
        let block = self.instructions().ok().flatten()?;
        let step = block.steps().find(|(_, step)| step.view.as_deref() == Some(active)).map(|(_, step)| step.id.clone());
        step
    }

    /// The presentation of the step whose view is shown, over the scene's
    /// solids.
    fn instructions_shown_presentation(&self) -> Option<edit::Presentation> {
        let active = self.pmi_active_view()?;
        let block = self.instructions().ok().flatten()?;
        let (_, step) = block.steps().find(|(_, step)| step.view.as_deref() == Some(active))?;
        let solids: Vec<String> = self.scene.solids().iter().map(|solid| solid.name.clone()).collect();
        edit::presentation(&block, &step.id, &solids).ok()
    }

    /// What automation and the panes read: the block, the step being shown,
    /// the steps left empty and the step count.
    pub fn instructions_state_json(&self) -> Value {
        match self.instructions() {
            Ok(Some(block)) => {
                let shown = self.pmi_active_view().and_then(|view| {
                    block.steps().find(|(_, s)| s.view.as_deref() == Some(view)).map(|(_, s)| s.id.clone())
                });
                json!({
                    "instructions": block.to_value(),
                    "steps": block.step_count(),
                    "wizardPending": self.instructions_wizard_pending,
                    "viewsPending": self.instructions_views_pending,
                    "shownStep": shown,
                    "ghosted": self.context_scene.solids().iter().map(|ghost| ghost.name.as_str()).collect::<Vec<_>>(),
                    "insertion": self.instructions_insertion_fraction(),
                    "emptySteps": edit::empty_steps(&block),
                    "unplanned": self.instructions_unplanned().into_iter().map(|(occurrence, part)| json!({ "occurrence": occurrence, "part": part })).collect::<Vec<_>>(),
                    "wrapper": self.instructions_wrapper().ok().map(|(id, part)| json!({ "id": id, "part": part })),
                })
            }
            Ok(None) => json!({ "instructions": null }),
            Err(error) => json!({ "instructions": null, "error": error }),
        }
    }
}

/// The start offset of a Line art step's derived insertion (see
/// [`EngineState::instructions_step_arrows`]): straight up the view's `up`
/// by twice the added parts' height along it, and at least a third of
/// everything the step shows, so a flat part still travels visibly. `None`
/// when the step places nothing over anything (its first part just
/// appears) or adds nothing drawn.
pub(crate) fn default_start_offset(
    presentation: &edit::Presentation,
    boxes: &std::collections::HashMap<String, crate::camera::Aabb>,
    up: [f64; 3],
) -> Option<[f64; 3]> {
    if presentation.placed.is_empty() || presentation.adding.is_empty() {
        return None;
    }
    let length = (up[0] * up[0] + up[1] * up[1] + up[2] * up[2]).sqrt();
    if !(length > 1e-12) {
        return None;
    }
    let up = up.map(|v| v / length);
    let union = |names: &[String]| {
        let mut bbox = crate::camera::Aabb::empty();
        for name in names {
            if let Some(solid) = boxes.get(name) {
                bbox.union(solid);
            }
        }
        bbox
    };
    let adding = union(&presentation.adding);
    if adding.is_empty() {
        return None;
    }
    let mut shown = union(&presentation.placed);
    shown.union(&adding);
    let along = |b: &crate::camera::Aabb| {
        let size = b.size();
        size[0] * up[0].abs() + size[1] * up[1].abs() + size[2] * up[2].abs()
    };
    let size = shown.size();
    let diagonal = (size[0] * size[0] + size[1] * size[1] + size[2] * size[2]).sqrt();
    let distance = (2.0 * along(&adding)).max(diagonal / 3.0);
    (distance.is_finite() && distance > 0.0).then(|| up.map(|v| v * distance))
}

/// A Line art step's derived arrows: one per occurrence the step and its
/// group add, each starting [`default_start_offset`] away.
pub(crate) fn default_arrows(
    step: &Step,
    presentation: &edit::Presentation,
    boxes: &std::collections::HashMap<String, crate::camera::Aabb>,
    up: [f64; 3],
) -> Vec<Arrow> {
    let Some(offset) = default_start_offset(presentation, boxes, up) else { return Vec::new() };
    let distance = (offset[0] * offset[0] + offset[1] * offset[1] + offset[2] * offset[2]).sqrt();
    let mut occurrences = Vec::new();
    step.occurrences_with_descendants(&mut occurrences);
    // A sub-assembly's occurrence stands for its parts: one arrow for it.
    occurrences
        .iter()
        .filter(|occurrence| !occurrences.iter().any(|other| other != *occurrence && occurrence.starts_with(&format!("{other}:"))))
        .map(|occurrence| Arrow { occurrence: occurrence.to_string(), direction: offset.map(|v| -v / distance), distance })
        .collect()
}
