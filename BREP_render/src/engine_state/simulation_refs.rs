//! A clip's parts picked in the viewport: the Simulation pane's forms are the
//! app's schema-driven dialogs, and a visibility or transparency clip's parts
//! are a `reference_selection` field — the same pick every feature dialog
//! runs ([`EngineState::begin_ref_select`] and kin). This is its lane
//! ([`RefSelectTarget::Simulation`]): begin on a clip's field, and on Finish
//! write the picked names into the clip through `simulation_patch` as ONE
//! undo step. Nothing rolls (a clip names the finished assembly, which is
//! what is on screen) and nothing re-runs.

use super::{EngineState, RefSelectState, RefSelectTarget};
use crate::engine_state::selection_filter::SelectionFilter;
use serde_json::json;

impl EngineState {
    /// Enter reference selection for clip `clip`'s list field at `path`
    /// (`["occurrences"]`). The filter is the schema's (`SOLID`, `COMPONENT`:
    /// a click on component geometry answers as its component).
    pub fn begin_ref_select_for_simulation(
        &mut self,
        clip: &str,
        path: Vec<String>,
        label: String,
        filter: Vec<String>,
        multiple: bool,
        seed_names: Vec<String>,
    ) {
        let restore_index = self.history.rollback();
        self.selection_filter = SelectionFilter::from_ref_filter(&filter);
        self.ref_select = Some(RefSelectState {
            feature_id: clip.to_string(),
            path,
            label,
            filter,
            multiple,
            names: seed_names,
            restore_index,
            target: RefSelectTarget::Simulation,
            picks: 0,
            claimed: false,
            finish_requested: false,
        });
        self.sync_ref_select_emphasis();
    }

    /// Commit a finished clip pick: every name as the component that holds it
    /// (a solid `ACOMP2:Part` stands for `ACOMP2`; a component pick is one
    /// already), without repeats, in pick order, written at `path`'s field
    /// of the clip as one undo step. The clip's study is found by the clip's
    /// id (ids are unique across the block).
    pub(crate) fn simulation_commit_refs(&mut self, clip: &str, path: &[String], names: &[String]) -> Result<(), String> {
        let block = self.simulation()?.ok_or("the document has no motion study")?;
        let study = block
            .studies
            .iter()
            .find(|study| study.clips.iter().any(|c| c.id == clip))
            .map(|study| study.id.clone())
            .ok_or_else(|| format!("no clip '{clip}'"))?;
        let mut occurrences: Vec<String> = Vec::new();
        for name in names {
            let occurrence = self.component_of_solid(name).unwrap_or_else(|| name.clone());
            if !occurrence.is_empty() && !occurrences.contains(&occurrence) {
                occurrences.push(occurrence);
            }
        }
        let field = path.last().map(String::as_str).unwrap_or("occurrences");
        self.simulation_patch(&study, Some(clip), &json!({ field: occurrences }), None)?;
        // A pick is a discrete change (it ends with Finish): the next edit of
        // the same clip's field is its own undo step.
        self.history.break_coalescing();
        Ok(())
    }
}

