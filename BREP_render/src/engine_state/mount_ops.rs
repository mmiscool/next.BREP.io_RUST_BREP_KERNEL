//! The MOUNT FRAME helpers that read the model: a part's mount set from a
//! pick (`brep_kernel::part_mount`).
//!
//! A pick is a helper that computes numbers ONCE. The mount block stores
//! the resolved pose and never the name, so nothing re-resolves when the part
//! changes. Each pick composes onto the current mount and is one undo step,
//! written through `History::set_mount_block`.

use super::*;

/// Which helper a mount pick is for.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MountPick {
    /// One vertex, edge or face point: it goes to the footprint origin, in
    /// the footprint's plane (`brep_kernel::mount_origin_to`).
    Origin,
    /// One planar face (or datum plane): it becomes the seating plane, its
    /// outward normal down and its plane at z = 0
    /// (`brep_kernel::mount_seated_on`).
    Seat,
}

impl MountPick {
    /// The picker's heading.
    pub fn label(self) -> &'static str {
        match self {
            MountPick::Origin => "Put this point on the footprint origin",
            MountPick::Seat => "Seat the part on this face",
        }
    }

    /// The pick kinds the helper admits. SOLID is offered to neither: a
    /// click meant for a face that landed on the body would name its centre.
    pub fn filter(self) -> Vec<String> {
        let kinds: &[&str] = match self {
            MountPick::Origin => &["VERTEX", "EDGE", "FACE"],
            MountPick::Seat => &["FACE", "PLANE"],
        };
        kinds.iter().map(|kind| (*kind).to_string()).collect()
    }
}

impl EngineState {
    /// Enter the reference picker for a mount helper. Nothing rolls back: a
    /// mount names the finished part, which is what is on screen.
    pub fn begin_ref_select_for_mount(&mut self, pick: MountPick) {
        self.disarm_transform();
        let filter = pick.filter();
        self.selection_filter = SelectionFilter::from_ref_filter(&filter);
        self.ref_select = Some(RefSelectState {
            feature_id: brep_kernel::MOUNT_BLOCK.to_string(),
            path: vec![brep_kernel::MOUNT_BLOCK.to_string()],
            label: pick.label().to_string(),
            filter,
            multiple: false,
            names: Vec::new(),
            restore_index: self.history.rollback(),
            target: RefSelectTarget::Mount { pick },
            picks: 0,
            claimed: false,
            finish_requested: false,
        });
        self.sync_ref_select_emphasis();
    }

    /// The mount a pick of `name` asks for, composed onto the current one.
    /// The name is resolved against the scene on screen (the rolled-to run).
    pub fn mount_from_pick(&self, pick: MountPick, name: &str) -> Result<Option<serde_json::Value>, String> {
        let current = self.history.mount_block().map(brep_kernel::mount_from_block).transpose()?;
        let request: HistoryRequest =
            serde_json::from_value(self.history.prefix_request()).map_err(|error| error.to_string())?;
        let _trace = crate::run_trace::span("mount_from_pick");
        let result = self.execute_plugin_history(&request);
        let mount = match pick {
            MountPick::Origin => {
                let point = brep_kernel::face_transform_reference_point(&result.results, name)?;
                brep_kernel::mount_origin_to(current.as_ref(), point)?
            }
            MountPick::Seat => {
                let (origin, normal) = brep_kernel::mount_reference_plane(&result.results, name)?;
                brep_kernel::mount_seated_on(current.as_ref(), origin, normal)?
            }
        };
        Ok(brep_kernel::mount_block_of(&mount))
    }

    /// Finish a mount pick: write what it asks for as one undo step, or say
    /// why it could not. A pick of nothing changes nothing.
    pub(super) fn mount_commit_pick(&mut self, pick: MountPick, name: Option<&str>) {
        let Some(name) = name.filter(|name| !name.trim().is_empty()) else {
            return;
        };
        match self.mount_from_pick(pick, name) {
            Ok(block) if block.as_ref() != self.history.mount_block() => self.history.set_mount_block(block, None),
            Ok(_) => {}
            Err(error) => self.push_notice(format!("{}: {error}", pick.label())),
        }
    }
}

