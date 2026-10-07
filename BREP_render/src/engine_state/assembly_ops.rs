use super::*;
use serde_json::Value;

// Assembly sessions are thread-local. The runner reads its session at the end of
// every run and ships it (`pipeline::AssemblySync`); the main thread ADOPTS that
// — the component projection, the solved constraint state, the statuses, the DOF
// summary and the overlay rows — instead of executing the history a second time.
// Componentless histories ship nothing and the projection is cleared.
//
// The ONE main-side replay left is `ensure_assembly_session`: a constraint
// mutation auto-solves against the resident geometry of the thread that asks, and
// the kernel's registry is thread-local, so the session has to exist HERE. That
// is per USER constraint edit, not per run. Mutations then call the kernel ABI,
// adopt the result with an undo checkpoint, and re-snapshot so the panels read
// the mutation before the rerun's reply lands. Auto-solve reruns the display
// history; otherwise manual Solve does.

/// What a component-insert names: an EXISTING parts-library entry (skip the
/// store read — just add an instance) or a NEW part payload to hand to
/// `add_part_to_library` (which dedups by sourceKey+signature and returns the
/// EFFECTIVE entry name the instance must reference).
pub enum ComponentInsert<'a> {
    Existing {
        part_name: &'a str,
    },
    New {
        name: &'a str,
        source_key: &'a str,
        source_signature: &'a str,
        document_json: &'a str,
    },
}

/// A STABLE content signature of a document: a sorted-key JSON walk hashed
/// with the (fixed-key, cross-process-deterministic) std SipHash. The ONE
/// signature fn, at the ENGINE altitude so app-side and engine-side writers
/// share one copy: the app's insert flow (`panels::file`), the edit-in-place
/// Finish (`panels::assembly_edit::refresh_library_entry`) and the
/// update-components comparison (`panels::update_components`) all write/compare
/// a parts-library `sourceSignature` with THIS function — a freshly inserted,
/// unchanged part must always compare up-to-date, which a second copy would
/// break the moment either drifted.
/// Uses the kernel's shared sorted-key hash, also used for in-memory library
/// fingerprints. Invalid JSON retains the raw-text hashing fallback.
pub fn document_signature(doc_json: &str) -> String {
    use std::hash::{Hash, Hasher};
    let hash = match serde_json::from_str::<serde_json::Value>(doc_json) {
        Ok(value) => brep_kernel::stable_json_hash(&value),
        Err(_) => {
            let mut hasher = std::collections::hash_map::DefaultHasher::new();
            doc_json.hash(&mut hasher);
            hasher.finish()
        }
    };
    format!("{hash:016x}")
}

/// Rigid inverse `[Rᵀ | −Rᵀ·t]` of a component pose — used to express a picked
/// WORLD point in COMPONENT-LOCAL coordinates (the vertex-ref contract).
fn rigid_inverse_point(transform: &brep_kernel::AffineTransform, world: [f64; 3]) -> [f64; 3] {
    let m = &transform.elements;
    let d = [world[0] - m[3], world[1] - m[7], world[2] - m[11]];
    [
        m[0] * d[0] + m[4] * d[1] + m[8] * d[2],
        m[1] * d[0] + m[5] * d[1] + m[9] * d[2],
        m[2] * d[0] + m[6] * d[1] + m[10] * d[2],
    ]
}

impl EngineState {
    // --- read surface ------------------------------------------------------

    /// Whether the current document is an ASSEMBLY document: any ACOMP-typed
    /// feature, or a present `assembly` constraint block. Componentless
    /// documents skip every main-side sync (zero cost for modeling files).
    pub fn history_has_assembly(&self) -> bool {
        let has_acomp = (0..self.history.len()).any(|index| {
            matches!(
                self.history.feature_type(index).as_deref(),
                Some("ACOMP") | Some("ASSEMBLY COMPONENT")
            )
        });
        has_acomp
            || self
                .history
                .assembly_block()
                .map(|block| !block.is_null())
                .unwrap_or(false)
    }

    /// The scene's component records (deterministic id order) as of the last
    /// applied run — the Assembly Structure tree's projection source. A VIEW
    /// over the scene, never an owning structure.
    pub fn assembly_components(&self) -> &[crate::pipeline::ComponentSnapshot] {
        &self.assembly_components
    }

    /// The assembly tail of the last applied run, or an empty stand-in for a
    /// componentless document / a document that has not run yet.
    fn assembly_sync(&self) -> std::borrow::Cow<'_, crate::pipeline::AssemblySync> {
        match &self.assembly_sync {
            Some(sync) => std::borrow::Cow::Borrowed(sync),
            None => std::borrow::Cow::Owned(crate::pipeline::AssemblySync::default()),
        }
    }

    /// The per-constraint status rows (`[{id, type, enabled, open, status,
    /// message, satisfied, error}]`) the run reported.
    pub fn assembly_statuses_value(&mut self) -> Value {
        match &self.assembly_sync().statuses {
            Value::Null => Value::Array(Vec::new()),
            value => value.clone(),
        }
    }

    /// The current `assembly` block (post-solve): `{constraints, idCounter}`.
    pub fn assembly_state_value(&mut self) -> Value {
        match &self.assembly_sync().state {
            // Before the first run there is no state at all; answer with the
            // empty block every caller expects rather than a bare null.
            Value::Null => serde_json::json!({ "constraints": [], "idCounter": 0 }),
            value => value.clone(),
        }
    }

    /// The last solve's DOF/diagnostics summary (`{ok, dof, rank, redundant,
    /// …}` — `movedSolids` may be absent; tolerate it).
    pub fn assembly_dof_value(&mut self) -> Value {
        self.assembly_sync().dof.clone()
    }

    /// Per-constraint overlay rows (world anchors/directions/status/value) —
    /// the constraints panel reads the evaluated value/unit for the
    /// distance/angle label suffix; lane G's viewport graphics read the rest.
    pub fn assembly_overlay_value(&mut self) -> Value {
        match &self.assembly_sync().overlay {
            Value::Null => Value::Array(Vec::new()),
            value => value.clone(),
        }
    }

    /// The parts-library entry names currently resident (insert-flow "existing
    /// entries first" list). The main-side store, seeded by document ingest and
    /// kept current by the runs that heal or GC it (`apply_run_output` installs
    /// the store a run moved), and grown by [`insert_component`].
    pub fn parts_library_names(&mut self) -> Vec<String> {
        serde_json::from_str::<Value>(&brep_kernel::parts_library_json())
            .ok()
            .and_then(|value| {
                value
                    .as_object()
                    .map(|map| map.keys().cloned().collect::<Vec<_>>())
            })
            .unwrap_or_default()
    }

    // --- the adopt + fold (pose-authority contract) -------------------------

    /// Adopt the assembly tail the runner shipped with the applied run: the
    /// component projection, and the post-solve constraint state folded back
    /// into the document (silent — a solver write-back is not a user edit; the
    /// pose/`isFixed` half was already folded in `apply_run_output`, and the
    /// parts-library store the run moved was installed there too).
    /// Componentless documents just clear the projection.
    ///
    /// This is the call that used to be `sync_assembly`, a second full
    /// `execute_history` on the UI thread. See `pipeline::AssemblySync`.
    pub(crate) fn adopt_assembly_sync(&mut self) {
        let Some(sync) = &self.assembly_sync else {
            self.assembly_components.clear();
            // No ACOMP references left ⇒ any partsLibrary block is orphaned
            // payload (the kernel GCs the store the same way at the end of
            // every run) — drop it so deleting the last instance never leaves
            // a dangling entry in the saved document. Gated on the WHOLE
            // history, not on the run: the run executes the rolled-to PREFIX,
            // so rolling back behind the first ACOMP ships no tail while the
            // document's components — and therefore its library — are still
            // very much there.
            if !self.history_has_assembly() {
                self.history
                    .set_parts_library(Value::Object(serde_json::Map::new()));
            }
            return;
        };
        self.assembly_components = sync.components.clone();
        let state = sync.state.clone();
        self.fold_assembly_state(state, false);
    }

    /// Write the post-solve `assembly` block into the document — the state half
    /// of the pose-authority write-back, which the kernel's
    /// `assembly_apply_document_json` performs alongside the pose half.
    /// `checkpoint` = true for a USER mutation (undoable), false for the silent
    /// post-run adopt. A null state (no session / no constraints yet) writes
    /// nothing.
    fn fold_assembly_state(&mut self, state: Value, checkpoint: bool) {
        if state.is_null() {
            return;
        }
        if self.history.assembly_block() == Some(&state) {
            return;
        }
        self.history.set_assembly_block(state, checkpoint);
    }

    /// Make sure a main-side kernel assembly SESSION exists for the last applied
    /// run — the one thing the runner's reply cannot carry.
    ///
    /// A constraint mutation auto-solves immediately, against the resident
    /// geometry of whichever thread holds it, and the kernel's registry and
    /// session are both thread-local; so a mutation needs the history executed
    /// HERE. That is per USER constraint edit — adding, editing, enabling,
    /// reordering or removing a mate, inferring constraints, or pressing Solve —
    /// not per run, which is the whole difference from the old `sync_assembly`.
    /// Cheap no-op when the session is already current, and for a componentless
    /// document (nothing to solve).
    pub(crate) fn ensure_assembly_session(&mut self) {
        if self.assembly_session_generation == Some(self.applied_generation) {
            return;
        }
        if !self.history_has_assembly() {
            return;
        }
        let request: brep_kernel::HistoryRequest =
            match serde_json::from_value(self.run_request_value()) {
                Ok(request) => request,
                Err(_) => return, // unparseable mid-edit document — retry next frame
            };
        let mut _trace = crate::run_trace::span("assembly_session");
        let result = self.execute_plugin_history(&request);
        if let Some(trace) = _trace.as_mut() {
            trace.result(&result);
        }
        self.assembly_session_generation = Some(self.applied_generation);
    }

    /// Re-read the main-side session into [`Self::assembly_sync`] after a
    /// MUTATION solved against it, so the panels (which read the snapshot, not
    /// the kernel) show the mutation immediately instead of lagging until the
    /// rerun's reply lands. The component projection is unchanged by a
    /// constraint mutation — only the solve moves poses, and the rerun brings
    /// those — so it is carried over.
    pub(crate) fn snapshot_assembly_from_session(&mut self) {
        let parse = |json: String| serde_json::from_str::<Value>(&json).unwrap_or(Value::Null);
        let components = self.assembly_components.clone();
        self.assembly_sync = Some(crate::pipeline::AssemblySync {
            components,
            state: parse(brep_kernel::assembly_state_json()),
            statuses: parse(brep_kernel::assembly_statuses_json()),
            dof: parse(brep_kernel::assembly_dof_json()),
            overlay: parse(brep_kernel::assembly_overlay_json()),
            parts_library: None,
        });
    }

    /// Run the document fold: `assembly_apply_document_json(document)` →
    /// adopt the returned document (assembly block replaced; solved poses +
    /// isFixed folded into features by `inputParams.id`). `checkpoint` = true
    /// for USER mutations (undoable), false for the silent post-run fold.
    /// A missing session (no run yet) is tolerated silently.
    fn apply_assembly_fold(&mut self, checkpoint: bool) {
        // The fold only rewrites the `assembly` block and per-feature
        // `inputParams` — it never reads `partsLibrary` — so hand it the
        // document WITHOUT the library. Otherwise every edit serialized,
        // parsed and re-serialized the whole embedded part payload three times
        // over for nothing. `adopt_document` keeps the field when the adopted
        // document carries no block, so the library survives the round trip.
        let document = self.history.request_json_without_parts_library();
        match brep_kernel::assembly_apply_document_impl(&document) {
            Ok(folded) => {
                let adopted = if checkpoint {
                    self.history.adopt_document_checkpointed(&folded)
                } else {
                    self.history.adopt_document(&folded)
                };
                if let Err(error) = adopted {
                    self.push_notice(format!("assembly fold failed: {error}"));
                }
                // The panels read the SHIPPED snapshot, not the kernel, so a
                // mutation that solved against the live session has to refresh
                // it — otherwise the constraint the user just added shows its
                // pre-solve status until the rerun's reply lands a frame or more
                // later. ONLY on the Ok arm: with no session the kernel's reads
                // answer with defaults, which would blank a good snapshot.
                self.snapshot_assembly_from_session();
            }
            Err(_) => {} // no session yet (fresh document before its first run)
        }
    }

    /// Shared tail of every USER constraint mutation: fold (checkpointed) and,
    /// when auto-solve is on, re-run the display history so the viewport
    /// re-poses (changed ACOMP transforms dirty their fingerprints → those
    /// instances re-execute + re-tessellate; the runner's tail re-solves).
    ///
    /// `pub(super)` because a constraint mutation also arrives from the VIEWPORT:
    /// an overlay handle drag commits on release
    /// ([`EngineState::constraint_drag_release`]) and needs the same two things
    /// this gives every panel edit — an undo entry for the drag, and a rerun
    /// that re-keys the moved instance's cache entry to its new pose.
    pub(super) fn after_constraint_mutation(&mut self) {
        self.apply_assembly_fold(true);
        if self.settings.assembly_auto_solve {
            self.rerun_history();
        } else {
            self.dirty = true;
        }
    }

    // --- constraint mutations (kernel ABI + fold + optional rerun) ----------

    /// Add a constraint (`params_json` = inputParams; the kernel mints the id
    /// from the type's short name when absent). Returns the minted id.
    pub fn assembly_add_constraint(
        &mut self,
        constraint_type: &str,
        params_json: &str,
    ) -> Result<String, String> {
        self.ensure_assembly_session();
        let reply =
            brep_kernel::assembly_add_constraint_impl(constraint_type, params_json)?;
        self.after_constraint_mutation();
        let id = serde_json::from_str::<Value>(&reply)
            .ok()
            .and_then(|value| value.get("id").and_then(|id| id.as_str()).map(String::from))
            .unwrap_or_default();
        Ok(id)
    }

    /// Replace a constraint's `inputParams` (the dialog commit).
    pub fn assembly_update_constraint(&mut self, id: &str, params_json: &str) -> Result<(), String> {
        self.ensure_assembly_session();
        brep_kernel::assembly_update_constraint_impl(id, params_json)?;
        self.after_constraint_mutation();
        Ok(())
    }

    /// Update WITHOUT the rerun tail — for callers whose own continuation
    /// re-runs anyway (the ref-select Finish, whose `end_ref_select` reruns).
    /// Still folds (checkpointed) so the document is current before that run.
    pub(crate) fn assembly_update_constraint_no_rerun(
        &mut self,
        id: &str,
        params_json: &str,
    ) -> Result<(), String> {
        self.ensure_assembly_session();
        brep_kernel::assembly_update_constraint_impl(id, params_json)?;
        self.apply_assembly_fold(true);
        Ok(())
    }

    /// Delete a constraint.
    pub fn assembly_remove_constraint(&mut self, id: &str) -> Result<(), String> {
        self.ensure_assembly_session();
        brep_kernel::assembly_remove_constraint_impl(id)?;
        self.after_constraint_mutation();
        Ok(())
    }

    /// Enable/disable a constraint (the row checkbox).
    pub fn assembly_set_constraint_enabled(&mut self, id: &str, enabled: bool) -> Result<(), String> {
        self.ensure_assembly_session();
        brep_kernel::assembly_set_constraint_enabled_impl(id, enabled)?;
        self.after_constraint_mutation();
        Ok(())
    }

    /// Persist WHICH constraint has its dialog open (view state — SILENT fold,
    /// no solve, no rerun, no undo entry). An open COUNTS
    /// ([`Self::assembly_constraint_opens`]) — including a re-open of the
    /// constraint already open, which is what the dialog door needs to surface
    /// the pane for a label click on a form the user cannot see. ACCORDION: at
    /// most ONE constraint is open — opening one first closes every other inside the same silent fold,
    /// so the panel toggle, the context bar's add-from-selection, and a viewport
    /// label click all converge on a single open constraint. The constraints
    /// PANEL reads this flag as its mode: open one and its form replaces the
    /// tree (`panels::assembly_constraints`), which is why every one of those
    /// surfaces opens the dialog without knowing the panel exists.
    pub fn assembly_set_constraint_open(&mut self, id: &str, open: bool) -> Result<(), String> {
        self.ensure_assembly_session();
        if open {
            let others: Vec<String> =
                serde_json::from_str::<Value>(&brep_kernel::assembly_state_json())
                    .ok()
                    .and_then(|state| {
                        state.get("constraints").and_then(|l| l.as_array()).map(|entries| {
                            entries
                                .iter()
                                .filter_map(|entry| {
                                    let cid =
                                        entry.get("inputParams")?.get("id")?.as_str()?;
                                    let is_open =
                                        entry.get("open").and_then(|v| v.as_bool()).unwrap_or(false);
                                    (is_open && cid != id).then(|| cid.to_string())
                                })
                                .collect()
                        })
                    })
                    .unwrap_or_default();
            for other in others {
                brep_kernel::assembly_set_constraint_open_impl(&other, false)?;
            }
        }
        brep_kernel::assembly_set_constraint_open_impl(id, open)?;
        if open {
            self.constraint_opens = self.constraint_opens.wrapping_add(1);
        }
        self.apply_assembly_fold(false);
        Ok(())
    }

    /// How many constraint dialogs this engine has OPENED through
    /// [`Self::assembly_set_constraint_open`] — the app's dialog door reads it
    /// beside [`Self::assembly_open_constraint`] so a re-open of the constraint
    /// already open is still an open (the id alone does not move: a viewport
    /// label click on the constraint whose form is up, made precisely because
    /// the user is on another tab).
    ///
    /// It counts the OPENS, not the flag's history. The flag itself lives in
    /// the kernel and a freshly minted constraint arrives `open: true` without
    /// passing through the setter — that one needs no count, because a new
    /// constraint brings a new id and the door's id half already sees it.
    pub fn assembly_constraint_opens(&self) -> u64 {
        self.constraint_opens
    }

    /// The id of the constraint whose DIALOG is open, or `None`.
    ///
    /// The accordion keeps at most one (`assembly_set_constraint_open` closes
    /// every other), so this is the panel's mode as ONE value. Reads the same
    /// two fields the Constraints panel's row builder reads — `inputParams.id`
    /// and `open` — off the borrowed fold state, without cloning the whole
    /// assembly block the way `assembly_state_value` does: the app's dialog
    /// door calls this every frame.
    pub fn assembly_open_constraint(&self) -> Option<String> {
        self.assembly_sync()
            .state
            .get("constraints")?
            .as_array()?
            .iter()
            .find(|entry| entry.get("open").and_then(Value::as_bool).unwrap_or(false))
            .and_then(|entry| entry.get("inputParams")?.get("id")?.as_str())
            .map(str::to_string)
    }

    /// Reorder a constraint to `index` (drag-reorder).
    pub fn assembly_move_constraint(&mut self, id: &str, index: usize) -> Result<(), String> {
        self.ensure_assembly_session();
        brep_kernel::assembly_move_constraint_impl(id, index)?;
        self.after_constraint_mutation();
        Ok(())
    }

    // --- automatic inference (the Auto Constraints button) ------------------

    /// The constraint types the inference lane can produce — `[{type, label,
    /// icon, longName, detects, defaultOn}]`, straight off the kernel's rule
    /// table. The dialog lists THIS; it keeps no roster of its own, so a rule
    /// added in the kernel shows up as a row without an app change.
    pub fn assembly_inferable_types(&self) -> Value {
        serde_json::from_str(&brep_kernel::assembly_inferable_types_json())
            .unwrap_or_else(|_| Value::Array(Vec::new()))
    }

    /// SCAN the placed components for the constraints their placement implies,
    /// creating nothing. `options` is the kernel's `InferOptions` JSON (`{}` for
    /// the defaults; the dialog sends the ticked `types`).
    pub fn assembly_infer_constraints(&mut self, options: &str) -> Value {
        self.ensure_assembly_session();
        serde_json::from_str(&brep_kernel::assembly_infer_constraints_json(options))
            .unwrap_or(Value::Null)
    }

    /// Scan and CREATE, as one undoable step: the kernel adds every accepted
    /// candidate and solves the batch once, then the usual mutation tail folds
    /// the result into the document and re-runs the display. A scan that found
    /// nothing skips the tail entirely (no checkpoint, no rerun).
    pub fn assembly_apply_inferred_constraints(&mut self, options: &str) -> Value {
        self.ensure_assembly_session();
        let reply: Value =
            serde_json::from_str(&brep_kernel::assembly_apply_inferred_constraints_json(options))
                .unwrap_or(Value::Null);
        let created = reply
            .get("created")
            .and_then(|value| value.as_array())
            .map(|list| list.len())
            .unwrap_or(0);
        if created > 0 {
            self.after_constraint_mutation();
        }
        reply
    }

    /// Manual solve (the panel's Solve button): solve the session, fold, and
    /// ALWAYS re-run the display (that is the point of pressing Solve — it
    /// works with auto-solve disabled).
    pub fn assembly_run_solve(&mut self) -> Result<(), String> {
        self.ensure_assembly_session();
        // Nothing to solve without components — bail BEFORE the kernel solve:
        // an empty assembly solving to a no-op is the correct result. (This
        // bail was once also the only thing between a missing session and a
        // native abort; the `_impl` doors now return that error as text.)
        if self.assembly_components().is_empty() {
            return Ok(());
        }
        brep_kernel::assembly_run_solve_impl()?;
        self.apply_assembly_fold(true);
        self.rerun_history();
        Ok(())
    }

    // --- component actions (route to the owning ACOMP feature) --------------

    /// Insert a component instance (the palette/insert flow): resolve the
    /// parts-library entry (add or reuse), refresh the document's
    /// `partsLibrary` block, append an ACOMP feature referencing the RETURNED
    /// effective part name with an identity transform, and re-run. The FIRST
    /// component of the document writes `isFixed: true` EXPLICITLY (dialog-
    /// visible); later instances write `false`. Returns the new feature id
    /// (`ACOMP<digits>` — the history's global counter mints that exact form).
    pub fn insert_component(&mut self, insert: ComponentInsert<'_>) -> Result<String, String> {
        let part_name = match insert {
            ComponentInsert::Existing { part_name } => part_name.to_string(),
            ComponentInsert::New {
                name,
                source_key,
                source_signature,
                document_json,
            } => match brep_kernel::add_part_to_library_impl(
                name,
                source_key,
                source_signature,
                document_json,
            ) {
                Ok(part_name) => part_name,
                Err(error) => {
                    // The Insert component dialog closes on Insert, so its
                    // status line never shows this; the toast is what the user
                    // sees (a file that is not a part document, for one).
                    self.push_notice(format!("Could not insert '{name}': {error}"));
                    return Err(error);
                }
            },
        };
        // The block must ride the request so the display runner ingests the
        // (new) entry on the very next run.
        if let Ok(library) = serde_json::from_str::<Value>(&brep_kernel::parts_library_json()) {
            self.history.set_parts_library(library);
        }
        let first = !(0..self.history.len()).any(|index| {
            matches!(
                self.history.feature_type(index).as_deref(),
                Some("ACOMP") | Some("ASSEMBLY COMPONENT")
            )
        });
        let id = self.history.next_feature_id("ACOMP");
        let feature = serde_json::json!({
            "type": "ACOMP",
            "inputParams": {
                "id": id,
                "partName": part_name,
                "transform": { "translate": [0, 0, 0], "rotateEulerDeg": [0, 0, 0] },
                "isFixed": first,
            },
            "persistentData": {}
        });
        self.add_feature(&feature.to_string())?;
        Ok(id)
    }

    /// Fix/Unfix a component: toggles the OWNING ACOMP feature's
    /// `inputParams.isFixed` (one truth, one undo lane) and re-runs.
    pub fn set_component_fixed(&mut self, component_id: &str, fixed: bool) -> Result<(), String> {
        let index = self
            .history
            .index_of(component_id)
            .ok_or_else(|| format!("no component feature '{component_id}'"))?;
        let mut params = self
            .history
            .feature_params(index)
            .unwrap_or_else(|| serde_json::json!({}));
        if let Some(map) = params.as_object_mut() {
            map.insert("isFixed".into(), Value::Bool(fixed));
        } else {
            return Err(format!("component '{component_id}': malformed inputParams"));
        }
        self.update_feature_params(component_id, &params.to_string())?;
        Ok(())
    }

    /// Select a component in the viewport: emphasis over exactly its member
    /// solids (the tree↔viewport sync lane; a viewport pick of any member
    /// lights the tree row through the same emphasis set).
    pub fn select_component(&mut self, component_id: &str) {
        self.select_components(&[component_id.to_string()]);
    }

    /// A component's member SOLID scene names (empty for an unknown id) — the
    /// selection/hover unit COMPONENT entries resolve to.
    pub fn component_member_solids(&self, component_id: &str) -> Vec<String> {
        self.assembly_components
            .iter()
            .filter(|record| record.id == component_id)
            .flat_map(|record| record.solids.iter().cloned())
            .collect()
    }

    /// TOGGLE a component in the current selection as ONE unit (the additive
    /// Ctrl/Cmd+click under COMPONENT promotion): when EVERY member solid is
    /// already selected the whole set deselects, otherwise the whole set joins
    /// the selection — the rest of the selection stays.
    pub fn toggle_component_selection(&mut self, component_id: &str) {
        let members = self.component_member_solids(component_id);
        if members.is_empty() {
            return;
        }
        let all_selected = members
            .iter()
            .all(|name| self.emphasis.selected_solids.contains(name));
        for name in members {
            if all_selected {
                self.emphasis.selected_solids.remove(&name);
            } else {
                self.emphasis.selected_solids.insert(name);
            }
        }
        self.emphasis.generation = self.emphasis.generation.wrapping_add(1);
        self.dirty = true;
    }

    /// Select SEVERAL components at once: emphasis over the union of their
    /// member solids (the interference window's row click highlights both
    /// participants of a pair through this).
    pub fn select_components(&mut self, component_ids: &[String]) {
        let members: Vec<String> = self
            .assembly_components
            .iter()
            .filter(|record| component_ids.iter().any(|id| id == &record.id))
            .flat_map(|record| record.solids.iter().cloned())
            .collect();
        let json = serde_json::json!({ "selected": { "solids": members } }).to_string();
        let _ = self.emphasis.apply_json(&json);
        self.dirty = true;
    }

    // --- constraint reference selection (the ref-select reuse) --------------

    /// Enter the modal reference picker for an ASSEMBLY CONSTRAINT's
    /// `elements`-style field (same widget, different commit target — Finish
    /// routes through [`Self::assembly_update_constraint_no_rerun`] instead of
    /// feature params). No roll-to-before: constraints pick against the FULL
    /// assembly.
    pub fn begin_ref_select_for_constraint(
        &mut self,
        constraint_id: &str,
        path: Vec<String>,
        label: String,
        filter: Vec<String>,
        multiple: bool,
        seed_names: Vec<String>,
    ) {
        let restore_index = self.history.rollback();
        self.selection_filter = SelectionFilter::from_ref_filter(&filter);
        self.ref_select = Some(RefSelectState {
            feature_id: constraint_id.to_string(),
            path,
            label,
            filter,
            multiple,
            names: seed_names,
            restore_index,
            target: RefSelectTarget::AssemblyConstraint,
        });
        self.sync_ref_select_emphasis();
    }

    /// Commit a finished constraint ref-select: read the constraint's params
    /// from the session state, write the picked names at the field path, and
    /// update (fold only — the caller's continuation reruns).
    pub(crate) fn assembly_commit_constraint_refs(
        &mut self,
        constraint_id: &str,
        path: &[String],
        names: &[String],
        multiple: bool,
    ) {
        let state = self.assembly_state_value();
        let Some(entry) = state
            .get("constraints")
            .and_then(Value::as_array)
            .and_then(|constraints| {
                constraints.iter().find(|entry| {
                    entry
                        .get("inputParams")
                        .and_then(|params| params.get("id"))
                        .and_then(Value::as_str)
                        == Some(constraint_id)
                })
            })
        else {
            self.push_notice(format!("unknown constraint '{constraint_id}'"));
            return;
        };
        let mut params = entry
            .get("inputParams")
            .cloned()
            .unwrap_or_else(|| serde_json::json!({}));
        let value = if multiple {
            Value::Array(names.iter().cloned().map(Value::String).collect())
        } else {
            Value::String(names.first().cloned().unwrap_or_default())
        };
        super::selection_ux::set_json_at(&mut params, path, value);
        if let Err(error) = self.assembly_update_constraint_no_rerun(constraint_id, &params.to_string())
        {
            self.push_notice(format!("constraint update failed: {error}"));
        }
    }

    /// Build the `{solidName}@x,y,z` COMPONENT-LOCAL vertex ref for a vertex
    /// pick on a component member (lane-E contract: world pick transformed by
    /// the owning component's inverse pose). `None` when the solid belongs to
    /// no component (vertex refs only exist for constraint selection).
    pub(crate) fn component_vertex_ref(
        &self,
        solid_name: &str,
        world_position: [f64; 3],
    ) -> Option<String> {
        let record = self.assembly_components.iter().find(|record| {
            record.solids.iter().any(|member| member == solid_name)
        })?;
        let local = rigid_inverse_point(&record.affine(), world_position);
        Some(format!(
            "{solid_name}@{},{},{}",
            local[0], local[1], local[2]
        ))
    }
}

